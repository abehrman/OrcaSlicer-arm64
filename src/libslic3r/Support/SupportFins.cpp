// Breakaway fin support (support type "fins").
//
// A fin is a thin solid wall standing on the bed beside the object, a small XY gap away from it,
// with a one-layer "tine" every few layers that bridges the gap and fuses into the object. The
// wall props the object up; the tines stop the object from leaning away from the wall, which
// matters most for parts printed tilted onto an edge. Because a tine is a single horizontal bead
// in the plane of the layer, it snaps off cleanly when the fin is bent away.
// The technique and its dimensions (0.2 mm standoff, one-layer tines about 0.5 mm wide,
// a 1 mm base at the bed) follow Slant3D's designed-in support fins (YouTube @slant3d,
// "How to Design Better Support Fins"). CNC Kitchen (YouTube @CNCKitchen, "Stop Printing Flat:
// The 45 Degree Secret for Stronger Parts") measured why printing tilted pays off: PLA printed at
// 45 degrees tested 27% stronger than upright. Fins are what make that orientation printable.
// The slicer-side design draws on Support Fins by Matthew Trahan (gittrahan,
// https://github.com/gittrahan/support-fins, MIT), which adds these fins to STL files;
// the default fin thickness and tine depth come from its geometry spec.
//
// Where the fins go follows Support Fins' placement rules and thresholds (MIT, Copyright (c) 2026
// gittrahan): walls under the overhangs, kept clear of the part, a row on each free edge, rim walls
// cut into short pieces, small holes left alone, and a judge that asks whether each bit of overhang is
// near a wall or bridged between two. The geometry here is built per layer from the slices instead of
// from 3D wall meshes, so fins go through the normal support toolpaths.
//
// For every object layer up to support_fin_height:
//  1. The fin envelope is the area under the object (the projection of all layers above) that is
//     clear of the object down to the bed, kept support_object_xy_distance off the object and
//     support_top_z_distance below it. Support blockers are subtracted.
//  2. Overhangs are what each slice adds over the one below, for faces up to kOverhangDeg from flat.
//     The roof of a hole narrower than kHoleDiameter with part under it is left alone.
//  3. Fins are laid per island of the envelope, fixed in XY for the whole object:
//     - cross rows run across the island's long axis, support_fin_spacing apart, where the island is
//       under an overhang. A piece shorter than kMinSpan at its base holds nothing and is dropped.
//     - rails run along the island's free edges (not against the object), kEdgeInset in from the edge,
//       on long islands (one per side: the anti-twist pair) and under level overhangs (rim walls).
//       They are cut into pieces at most kRimPiece long so they break away.
//     With support_fin_lean_side_only, only the fin columns on the side the object leans toward
//     are kept (see keep_lean_side()).
//     With support_fin_cross_spacing, fins also run across the main fins, bracing them into a scaffold.
//  4. Within the first kBaseHeight above the bed the fins are widened into a foot; within kTipHeight
//     under the object they narrow to one loop, so the top breaks off cleanly.
//  5. On tine layers, each fin that ends at the object is extended across the XY gap and
//     support_fin_tine_depth into the object slice. Tine rows are support_fin_tine_spacing apart; with
//     support_fin_tine_base_rows, extra rows are packed just above the foot, where the object is least
//     stable (Slant3D). Rails get a tine every kTinePitch along them.
//  6. With support_fin_interface_layers, the fin region right under the object is printed as
//     interface layers across the fin tops.
//  7. The judge marks the overhang no fin holds; with a fallback, those columns go to tree or normal support.
// The result is printed through the common support toolpath generator.

#include "SupportFins.hpp"
#include "SupportCommon.hpp"
#include "../ClipperUtils.hpp"
#include "../Layer.hpp"
#include "../Print.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <boost/log/trivial.hpp>
#include <tbb/parallel_for.h>

namespace Slic3r {

// Height of the widened fin foot on the bed, and its width as a multiple of the fin thickness.
static constexpr double kBaseHeight     = 1.0;
static constexpr double kBaseWidthRatio = 3.0;
// A fin taller than this many times its length is reported as slender.
static constexpr double kSlenderRatio   = 10.0;

// Extra rows near the bed: the first just above the foot, then gaps from kTineGapMin growing by
// kTineGapGrowth until they reach the regular tine spacing.
static constexpr double kTineGapMin    = 0.8;
static constexpr double kTineGapGrowth = 1.6;

// Placement and judge thresholds, from Support Fins (web/prop/config.js, judge.js, rim.js, fins/coverage.js).
// Faces flatter than this from horizontal are overhangs. Above Support Fins' 45 degrees so that a face
// at exactly 45 degrees, which slices into steps one layer wide, still counts.
static constexpr double kOverhangDeg  = 50.;
// Overhangs at least this flat make a level region, which gets rim walls (RIM.levelDeg).
static constexpr double kLevelDeg     = 10.;
// A hole narrower than this with part under it within kHoleFloor is left to bridge (HOLE_D).
static constexpr double kHoleDiameter = 6.;
static constexpr double kHoleFloor    = 8.;
// A cross row shorter than this at its base holds nothing (PROP.minSpanShort).
static constexpr double kMinSpan      = 4.;
// Rails stand this far in from a free edge (PROP.edgeInset) and are cut into pieces at most
// kRimPiece long with kRimGap between them (RIM.pieceLen, RIM.pieceGap).
static constexpr double kEdgeInset    = 0.6;
static constexpr double kRimPiece     = 40.;
static constexpr double kRimGap       = 1.5;
// Cross rows keep this far clear of a rail (PROP.sideClear).
static constexpr double kSideClear    = 0.35;
// An island this many times longer than wide, and at least kLongMin long, gets a rail on each side.
static constexpr double kLongAspect   = 3.;
static constexpr double kLongMin      = 20.;
// Fins narrow to one loop within this distance under the object (PROP.tipH).
static constexpr double kTipHeight    = 1.5;
// Tines along a rail, this far apart.
static constexpr double kTinePitch    = 3.;
// Judge: an overhang is held within kCantilever of a wall or of the part going down from it, or
// bridged between anchors at least kBridgeDeg apart whose distances add up to at most kSpan. Anchors
// are up to kAnchorDrop below it. Anything under kPlateZ is held by the bed (JUDGE, LOW_Z).
static constexpr double kCantilever   = 2.;
static constexpr double kSpan         = 12.;
static constexpr double kBridgeDeg    = 135.;
static constexpr int    kSectors      = 16;
static constexpr double kAnchorDrop   = 1.5;
static constexpr double kPlateZ       = 0.82;
static constexpr double kJudgeSample  = 1.;

std::vector<bool> fin_tine_layers(const PrintObject &object)
{
    const PrintObjectConfig &config = object.config();
    const auto   layers  = object.layers();
    const double spacing = config.support_fin_tine_spacing.value;
    std::vector<bool> rows(layers.size(), false);
    if (spacing <= 0.)
        return rows;
    // Regular rows, starting just above the foot.
    double next = kBaseHeight;
    double gap  = config.support_fin_tine_base_rows.value ? std::min(kTineGapMin, spacing) : spacing;
    for (size_t i = 1; i < layers.size(); ++ i)
        if (layers[i]->bottom_z() > next - EPSILON) {
            rows[i] = true;
            next    = layers[i]->print_z + gap;
            gap     = std::min(gap * kTineGapGrowth, spacing);
        }
    return rows;
}

// Strips are built horizontal in a frame rotated by -angle, then rotated back to run along `angle`.
static BoundingBox rotated_extents(const BoundingBox &bbox, double angle, coord_t margin)
{
    Polygon frame = bbox.polygon();
    frame.rotate(-angle);
    BoundingBox rbox = frame.bounding_box();
    rbox.offset(margin);
    return rbox;
}

static Polygon fin_strip(const BoundingBox &rbox, double angle, coord_t y, coord_t width)
{
    const coord_t half = width / 2;
    Polygon strip({ { rbox.min.x(), y - half }, { rbox.max.x(), y - half }, { rbox.max.x(), y + half }, { rbox.min.x(), y + half } });
    strip.rotate(angle);
    return strip;
}

// Parallel strips `width` wide with centers on a global grid of `pitch`, running along `angle`
// and covering `bbox`.
static Polygons fin_strips(const BoundingBox &bbox, double angle, coord_t pitch, coord_t width)
{
    const BoundingBox rbox = rotated_extents(bbox, angle, width);
    Polygons strips;
    for (coord_t k = coord_t(std::floor(double(rbox.min.y()) / pitch)); k * pitch <= rbox.max.y() + pitch; ++ k)
        strips.emplace_back(fin_strip(rbox, angle, k * pitch, width));
    return strips;
}

// Direction the fins run, in radians: across the long axis of the fin regions.
// Second moments are summed per island about the island's own centroid, so islands on
// opposite sides of the object do not read as one region spread across the object.
static double fin_angle(const std::vector<Polygons> &regions)
{
    double sxx = 0., syy = 0., sxy = 0.;
    for (const Polygons &region : regions)
        for (const ExPolygon &island : union_ex(region)) {
            double a = 0., cx = 0., cy = 0., ixx = 0., iyy = 0., ixy = 0.;
            auto add = [&](const Polygon &poly) {
                const Points &pts = poly.points;
                for (size_t i = 0; i < pts.size(); ++ i) {
                    const Point &p0 = pts[i], &p1 = pts[(i + 1) % pts.size()];
                    const double x0 = unscale<double>(p0.x()), y0 = unscale<double>(p0.y());
                    const double x1 = unscale<double>(p1.x()), y1 = unscale<double>(p1.y());
                    const double c  = x0 * y1 - x1 * y0;
                    a   += c;
                    cx  += (x0 + x1) * c;
                    cy  += (y0 + y1) * c;
                    ixx += (x0 * x0 + x0 * x1 + x1 * x1) * c;
                    iyy += (y0 * y0 + y0 * y1 + y1 * y1) * c;
                    ixy += (x0 * y1 + 2. * x0 * y0 + 2. * x1 * y1 + x1 * y0) * c;
                }
            };
            // Contours are CCW and holes CW, so holes subtract.
            add(island.contour);
            for (const Polygon &hole : island.holes)
                add(hole);
            a *= 0.5;
            if (a <= 0.)
                continue;
            cx /= 6. * a;
            cy /= 6. * a;
            sxx += ixx / 12. - a * cx * cx;
            syy += iyy / 12. - a * cy * cy;
            sxy += ixy / 24. - a * cx * cy;
        }
    // A round or square footprint has no long axis; keep a fixed direction instead of rounding noise.
    if (std::hypot(sxx - syy, 2. * sxy) < 1e-3 * (sxx + syy))
        return 0.;
    return 0.5 * std::atan2(2. * sxy, sxx - syy) + 0.5 * M_PI;
}

// Area-weighted centroid of `expolys` (holes subtract) in scaled coordinates; `area` receives the total area.
static Vec2d area_centroid(const ExPolygons &expolys, double &area)
{
    Vec2d sum = Vec2d::Zero();
    area = 0.;
    for (const ExPolygon &expoly : expolys) {
        // Polygon::area() is signed: contours are CCW (positive) and holes CW (negative).
        sum  += expoly.contour.area() * expoly.contour.centroid().cast<double>();
        area += expoly.contour.area();
        for (const Polygon &hole : expoly.holes) {
            sum  += hole.area() * hole.centroid().cast<double>();
            area += hole.area();
        }
    }
    return area > 0. ? Vec2d(sum / area) : sum;
}

// Keeps only the fin columns on the side the object leans toward: the side of its bed contact
// that its center of mass lies on. Those fins hold the object up in compression; a tine is too
// thin to hold it in tension from the other side. A balanced object keeps its largest column's side.
static void keep_lean_side(const LayerPtrs &layers, std::vector<Polygons> &regions)
{
    // Center of mass in XY: slice centroids weighted by slice area times layer height.
    Vec2d  com  = Vec2d::Zero();
    double mass = 0.;
    for (const Layer *layer : layers) {
        double      area;
        const Vec2d c = area_centroid(layer->lslices, area);
        com  += c * area * layer->height;
        mass += area * layer->height;
    }
    double      base_area;
    const Vec2d base = area_centroid(layers.front()->lslices, base_area);
    if (mass <= 0. || base_area <= 0.)
        return;
    com /= mass;

    // Fin regions only narrow going up, so the lowest one holds every fin column.
    auto lowest = std::find_if(regions.begin(), regions.end(), [](const Polygons &region) { return ! region.empty(); });
    if (lowest == regions.end())
        return;
    const ExPolygons columns = union_ex(*lowest);
    auto column_offset = [&base](const ExPolygon &column) { double area; return Vec2d(area_centroid({ column }, area) - base); };

    Vec2d lean = com - base;
    if (lean.norm() < scale_(0.5)) {
        const auto largest = std::max_element(columns.begin(), columns.end(),
            [](const ExPolygon &a, const ExPolygon &b) { return a.area() < b.area(); });
        lean = column_offset(*largest);
    }
    Polygons keep;
    for (const ExPolygon &column : columns)
        if (column_offset(column).dot(lean) > 0.)
            append(keep, to_polygons(column));
    if (keep.empty())
        return;
    // Upper regions sit inside the bed-level columns; allow for rounding at their edges.
    keep = offset(keep, float(scale_(0.1)));
    for (Polygons &region : regions)
        if (! region.empty())
            region = intersection(region, keep);
}

// Points along `lines` at most `step` mm apart, in mm.
static void sample_lines(const Polylines &lines, double step, std::vector<Vec2d> &out)
{
    for (const Polyline &line : lines)
        for (size_t i = 0; i + 1 < line.points.size(); ++ i) {
            const Vec2d  a = unscale(line.points[i]);
            const Vec2d  b = unscale(line.points[i + 1]);
            const int    n = std::max(1, int(std::ceil((b - a).norm() / step)));
            for (int k = 0; k < n; ++ k)
                out.emplace_back(a + (b - a) * (double(k) / n));
            if (i + 2 == line.points.size())
                out.emplace_back(b);
        }
}

// Anchor points bucketed on a grid of kSpan-sized cells, so a sample only looks at its own and the
// neighboring cells.
class AnchorGrid
{
public:
    explicit AnchorGrid(const std::vector<Vec2d> &points)
    {
        for (const Vec2d &p : points)
            m_cells[key(cell(p.x()), cell(p.y()))].push_back(p);
    }
    // Is `p` within kCantilever of an anchor, or bridged between two anchors at least kBridgeDeg
    // apart around it whose distances add up to at most kSpan?
    bool holds(const Vec2d &p) const
    {
        std::array<double, kSectors> nearest;
        nearest.fill(std::numeric_limits<double>::max());
        const int cx = cell(p.x()), cy = cell(p.y());
        for (int dx = -1; dx <= 1; ++ dx)
            for (int dy = -1; dy <= 1; ++ dy) {
                auto it = m_cells.find(key(cx + dx, cy + dy));
                if (it == m_cells.end())
                    continue;
                for (const Vec2d &a : it->second) {
                    const Vec2d  d    = a - p;
                    const double dist = d.norm();
                    if (dist <= kCantilever)
                        return true;
                    if (dist > kSpan)
                        continue;
                    const int sector = std::clamp(int(std::floor((std::atan2(d.y(), d.x()) + M_PI) / (2. * M_PI) * kSectors)), 0, kSectors - 1);
                    nearest[sector]  = std::min(nearest[sector], dist);
                }
            }
        const int apart = int(std::ceil(kBridgeDeg / 360. * kSectors));
        for (int a = 0; a < kSectors; ++ a)
            for (int b = a + apart; b <= a + kSectors - apart && b < kSectors; ++ b)
                if (nearest[a] + nearest[b] <= kSpan)
                    return true;
        return false;
    }

private:
    static int     cell(double v) { return int(std::floor(v / (kSpan + 1.))); }
    static int64_t key(int x, int y) { return (int64_t(x) << 32) ^ int64_t(uint32_t(y)); }
    std::unordered_map<int64_t, std::vector<Vec2d>> m_cells;
};

// The judge, after Support Fins' judge.js: samples each layer's overhang about kJudgeSample apart and
// returns, projected onto the bed, the overhang no fin holds. A sample is held by the bed (under kPlateZ),
// or by anchors up to kAnchorDrop below it: the tops of the fin walls (minus the top Z gap) and the edge
// of the part the overhang grows out of.
static Polygons judge_fins(const PrintObject &object, const std::vector<Polygons> &outlines,
    const std::vector<Polygons> &overhangs, const std::vector<Polygons> &walls, double top_gap, float min_width)
{
    const auto       layers     = object.layers();
    const size_t     num_layers = layers.size();
    const float      touch      = float(scale_(0.1));
    // Anchors per layer, in mm.
    std::vector<std::vector<Vec2d>> wall_anchors(num_layers), part_anchors(num_layers);
    tbb::parallel_for(tbb::blocked_range<size_t>(0, num_layers), [&](const tbb::blocked_range<size_t> &range) {
        for (size_t k = range.begin(); k < range.end(); ++ k) {
            if (! walls[k].empty())
                sample_lines(to_polylines(walls[k]), 0.5 * kJudgeSample, wall_anchors[k]);
            // The edge of slice k that the overhang above grows out of, where slice k is not itself
            // overhanging: a slope does not hold itself up.
            if (k + 1 < num_layers && ! overhangs[k + 1].empty()) {
                const Polygons root = diff(offset(overhangs[k + 1], touch), offset(overhangs[k], touch));
                if (! root.empty())
                    sample_lines(intersection_pl(to_polylines(outlines[k]), root), 0.5 * kJudgeSample, part_anchors[k]);
            }
        }
    });

    std::vector<Polygons> uncovered(num_layers);
    std::vector<double>   total(num_layers, 0.), held_area(num_layers, 0.);
    tbb::parallel_for(tbb::blocked_range<size_t>(1, num_layers), [&](const tbb::blocked_range<size_t> &range) {
        for (size_t i = range.begin(); i < range.end(); ++ i) {
            if (overhangs[i].empty())
                continue;
            const double bottom = layers[i]->bottom_z();
            if (bottom < kPlateZ)
                continue;
            std::vector<Vec2d> anchors;
            for (size_t k = i; k -- > 0 && layers[k]->print_z > bottom - kAnchorDrop - top_gap - EPSILON;) {
                if (layers[k]->print_z <= bottom + 0.05)
                    append(anchors, wall_anchors[k]);
                if (layers[k]->print_z > bottom - kAnchorDrop - EPSILON)
                    append(anchors, part_anchors[k]);
            }
            const AnchorGrid grid(anchors);
            const ExPolygons pieces = union_ex(overhangs[i]);
            std::vector<Vec2d> samples;
            sample_lines(to_polylines(to_polygons(pieces)), kJudgeSample, samples);
            for (const ExPolygon &piece : pieces) {
                const BoundingBox b = get_extents(piece);
                const coord_t     s = scaled<coord_t>(kJudgeSample);
                for (coord_t x = b.min.x() + s / 2; x < b.max.x(); x += s)
                    for (coord_t y = b.min.y() + s / 2; y < b.max.y(); y += s)
                        if (piece.contains(Point(x, y)))
                            samples.emplace_back(unscale(Point(x, y)));
            }
            Polygons held;
            const double half = 0.6 * kJudgeSample;
            for (const Vec2d &p : samples)
                if (grid.holds(p))
                    held.emplace_back(BoundingBox(Point::new_scale(p.x() - half, p.y() - half), Point::new_scale(p.x() + half, p.y() + half)).polygon());
            uncovered[i] = diff(overhangs[i], held);
            total[i]     = area(overhangs[i]);
            held_area[i] = total[i] - area(uncovered[i]);
        }
    });
    Polygons out;
    double   sum_total = 0., sum_held = 0.;
    for (size_t i = 0; i < num_layers; ++ i) {
        append(out, std::move(uncovered[i]));
        sum_total += total[i];
        sum_held  += held_area[i];
    }
    BOOST_LOG_TRIVIAL(info) << "Fin support - judge: " << (sum_total > 0. ? 100. * sum_held / sum_total : 100.) << "% of the overhang above the bed is held";
    return opening(union_(out), min_width);
}

bool generate_fin_support(PrintObject& object, Polygons* reserved_columns)
{
    const PrintObjectConfig &config         = object.config();
    const SlicingParameters &slicing_params = object.slicing_parameters();
    SupportParameters        support_params(object);
    const LayerPtrs         &layers         = object.layers();
    const size_t             num_layers     = layers.size();
    if (num_layers < 2)
        return false;

    BOOST_LOG_TRIVIAL(info) << "Fin support - start";

    // Object outlines with holes filled: fins never stand inside the object.
    std::vector<Polygons> outlines(num_layers);
    tbb::parallel_for(tbb::blocked_range<size_t>(0, num_layers), [&](const tbb::blocked_range<size_t> &range) {
        for (size_t i = range.begin(); i < range.end(); ++ i)
            for (const ExPolygon &expoly : layers[i]->lslices)
                outlines[i].emplace_back(expoly.contour);
    });

    std::vector<Polygons> blockers = object.slice_support_blockers();
    object.project_and_append_custom_facets(false, EnforcerBlockerType::BLOCKER, blockers);

    // 1. Fin regions, only for the layers up to the fin height.
    const coordf_t fin_height = config.support_fin_height.value;
    const coordf_t top_gap    = config.support_top_z_distance.value;
    size_t         num_fin_layers = num_layers;
    if (fin_height > 0.)
        while (num_fin_layers > 0 && layers[num_fin_layers - 1]->print_z > fin_height + EPSILON)
            -- num_fin_layers;
    // `shadow` is the projection of everything above a layer.
    std::vector<Polygons> shadow(num_fin_layers);
    {
        Polygons above;
        for (size_t i = num_layers; i -- > 0;) {
            if (i < num_fin_layers)
                shadow[i] = above;
            if (! outlines[i].empty())
                above = union_(above, outlines[i]);
            if (object.print()->canceled())
                return false;
        }
    }
    // A fin stands on the bed, so it may not pass through anything at or under its layer:
    // the object (`below`) or a support blocker (`blocked`).
    std::vector<Polygons> regions(num_layers);
    {
        Polygons below, blocked;
        for (size_t i = 0; i < num_fin_layers; ++ i) {
            below = union_(below, outlines[i]);
            if (i < blockers.size() && ! blockers[i].empty())
                blocked = union_(blocked, blockers[i]);
            const Layer &layer = *layers[i];
            if (! shadow[i].empty()) {
                Polygons keep_out = below;
                // Stop fins support_top_z_distance below any part of the object that hangs over them.
                for (size_t j = i + 1; j < num_layers && layers[j]->bottom_z() < layer.print_z + top_gap - EPSILON; ++ j)
                    append(keep_out, outlines[j]);
                const float gap = float(scale_(i == 0 ? support_params.gap_xy_first_layer : support_params.gap_xy));
                regions[i] = diff(shadow[i], offset(union_(keep_out), gap));
                if (! blocked.empty())
                    regions[i] = diff(regions[i], blocked);
            }
            Polygons().swap(shadow[i]);
            if (object.print()->canceled())
                return false;
        }
    }

    if (config.support_fin_lean_side_only.value)
        keep_lean_side(layers, regions);

    const coord_t  spacing     = support_params.support_material_flow.scaled_spacing();
    // Fins are filled with concentric loops, two lines per loop. Round the thickness to whole loops:
    // a leftover narrower than a line is not filled and leaves a slot down the middle of the fin.
    const coord_t  thickness   = 2 * spacing * std::max<coord_t>(1, coord_t(std::lround(scaled<double>(config.support_fin_thickness.value) / (2. * spacing))));
    const coord_t  pitch       = scaled<coord_t>(config.support_fin_spacing.value) + thickness;
    // A tine is one bead out and one bead back: two extrusions wide, never wider than the fin.
    const coord_t  tine_width  = std::min(thickness, 2 * spacing);
    const coord_t  foot_width  = coord_t(kBaseWidthRatio * thickness);
    // Pieces narrower than half an extrusion cannot be printed; drop them.
    const float    min_width   = 0.25f * float(spacing);
    const float    depth       = float(scale_(config.support_fin_tine_depth.value));
    const float    tine_reach  = float(scale_(support_params.gap_xy)) + depth + float(spacing);
    const coord_t  cross_pitch = config.support_fin_cross_spacing.value > 0. ? scaled<coord_t>(config.support_fin_cross_spacing.value) + thickness : 0;
    // A piece of wall this short (area / thickness) holds nothing.
    const double   min_piece   = kMinSpan * scale_(1.) * double(thickness);

    // 2. Overhangs: what each slice adds over the one below, for faces up to kOverhangDeg from flat.
    // The roof of a small hole, spanning a void narrower than kHoleDiameter with part under it, bridges.
    std::vector<Polygons> overhangs(num_layers), level_overhangs(num_layers), hole_roofs(num_layers);
    tbb::parallel_for(tbb::blocked_range<size_t>(1, num_layers), [&](const tbb::blocked_range<size_t> &range) {
        for (size_t i = range.begin(); i < range.end(); ++ i) {
            const double h = layers[i]->height;
            // A face kOverhangDeg from flat steps out h * cot(kOverhangDeg) per layer; keep the whole step
            // where it is at least that wide.
            const Polygons step     = diff(outlines[i], outlines[i - 1]);
            const Polygons overhang = opening(step, float(scale_(0.5 * h / std::tan(kOverhangDeg * M_PI / 180.))));
            if (overhang.empty())
                continue;
            // Voids narrower than a hole close up.
            const Polygons narrow_voids = closing(outlines[i - 1], float(scale_(0.5 * kHoleDiameter)));
            const double   floor_z      = layers[i]->bottom_z() - kHoleFloor;
            for (const ExPolygon &piece : union_ex(overhang)) {
                const Polygons p = to_polygons(piece);
                bool hole = area(intersection(p, narrow_voids)) > 0.9 * piece.area();
                if (hole) {
                    hole = false;
                    for (size_t k = i - 1; k -- > 0 && layers[k]->print_z > floor_z - EPSILON;)
                        if (! intersection(p, outlines[k]).empty()) {
                            hole = true;
                            break;
                        }
                }
                append(hole ? hole_roofs[i] : overhangs[i], p);
            }
            // A slice that grows by more than h * cot(kLevelDeg) at once is a level overhang.
            level_overhangs[i] = opening(overhangs[i], float(scale_(0.5 * h / std::tan(kLevelDeg * M_PI / 180.))));
        }
    });
    Polygons overhang_all, level_all;
    for (size_t i = 1; i < num_layers; ++ i) {
        append(overhang_all, overhangs[i]);
        append(level_all, level_overhangs[i]);
    }
    overhang_all = union_(overhang_all);
    level_all    = union_(level_all);
    const Polygons reach = offset(overhang_all, float(scale_(kCantilever)));
    // Small holes bridge; keep the fallback out of them, whether or not there are fins.
    auto reserve_holes = [&]() {
        Polygons holes;
        for (const Polygons &roof : hole_roofs)
            append(holes, roof);
        if (! holes.empty())
            append(*reserved_columns, offset(union_(holes), float(spacing)));
    };

    BoundingBox bbox;
    for (const Polygons &region : regions)
        if (! region.empty())
            bbox.merge(get_extents(region));
    if (! bbox.defined) {
        if (reserved_columns != nullptr) {
            reserved_columns->clear();
            reserve_holes();
        }
        return false;
    }

    // 3. Fins, laid per island of the lowest fin region (regions only narrow going up).
    const size_t first_region = size_t(std::find_if(regions.begin(), regions.end(), [](const Polygons &r) { return ! r.empty(); }) - regions.begin());
    const ExPolygons islands  = union_ex(regions[first_region]);
    // The part the island stands beside on the bed: its edge against that part is not free.
    Polygons bed_part;
    for (size_t i = 0; i <= first_region; ++ i)
        append(bed_part, outlines[i]);
    const float    inset     = float(scale_(kEdgeInset));
    const Polygons not_free  = offset(union_(bed_part), float(scale_(support_params.gap_xy_first_layer + 0.2)) + inset + float(thickness));
    // Rim walls are cut into pieces at most kRimPiece long.
    Polygons rim_cuts = fin_strips(bbox, 0., scaled<coord_t>(kRimPiece + kRimGap), scaled<coord_t>(kRimGap));
    append(rim_cuts, fin_strips(bbox, 0.5 * M_PI, scaled<coord_t>(kRimPiece + kRimGap), scaled<coord_t>(kRimGap)));

    // Walls and their feet, the one-loop band they narrow to at the top, and tine strips in two
    // directions (kept apart so crossing strips do not merge into one long piece).
    Polygons walls, feet, thin, tines_along, tines_across;
    double   interface_dir = 0., largest = 0.;
    for (const ExPolygon &island : islands) {
        const Polygons island_p = to_polygons(island);
        const double   angle    = fin_angle({ island_p });
        if (island.area() > largest) {
            largest       = island.area();
            interface_dir = angle;
        }
        Polygon frame = island.contour;
        frame.rotate(-angle);
        const BoundingBox rb = frame.bounding_box();
        BoundingBox rbox = rb;
        rbox.offset(foot_width);
        const double length = unscale<double>(rb.size().y());
        const double width  = unscale<double>(rb.size().x());

        // Rails: one along each free edge of a long island (the anti-twist pair, ends dropped),
        // and rim walls along the whole free edge under a level overhang.
        Polygons rails;
        const Polygons under   = intersection(island_p, overhang_all);
        const double   under_a = area(under);
        const bool     level   = under_a > 0. && area(intersection(under, level_all)) >= 0.5 * under_a;
        const bool     longish = length >= kLongAspect * width && length >= kLongMin;
        if (level || longish) {
            Polygons ring = diff(offset(island, - inset), offset(island, - inset - float(thickness)));
            ring = diff(ring, not_free);
            if (! level) {
                const coord_t m = inset + thickness + scaled<coord_t>(0.5);
                Polygon middle({ { rbox.min.x(), rb.min.y() + m }, { rbox.max.x(), rb.min.y() + m }, { rbox.max.x(), rb.max.y() - m }, { rbox.min.x(), rb.max.y() - m } });
                middle.rotate(angle);
                ring = intersection(ring, Polygons{ middle });
            }
            for (const ExPolygon &piece : union_ex(diff(ring, rim_cuts)))
                if (piece.area() >= min_piece)
                    append(rails, to_polygons(piece));
        }
        if (! rails.empty()) {
            append(feet, intersection(offset(rails, float(thickness)), island_p));
            if (thickness > 2 * spacing) {
                const float mid = inset + 0.5f * float(thickness);
                append(thin, intersection(rails, diff(offset(island, - mid + float(spacing)), offset(island, - mid - float(spacing)))));
            }
            const Polygons near_rails = offset(rails, tine_reach + float(thickness));
            const BoundingBox ib = get_extents(island_p);
            append(tines_along, intersection(fin_strips(ib, angle, scaled<coord_t>(kTinePitch), tine_width), near_rails));
            append(tines_across, intersection(fin_strips(ib, angle + 0.5 * M_PI, scaled<coord_t>(kTinePitch), tine_width), near_rails));
        }

        // Cross rows across the long axis, centered on the island, where it is under an overhang.
        // A row shorter than kMinSpan at its base is a dash that holds nothing, so it is dropped.
        const Polygons allowed = intersection(island_p, reach);
        const Polygons clear   = offset(rails, float(scale_(kSideClear)));
        const int      rows    = std::max(1, int(std::lround(double(rb.size().y()) / double(pitch))));
        bool           any     = ! rails.empty();
        auto add_row = [&](coord_t y, const Polygons &row) {
            append(walls, row);
            append(feet, intersection(Polygons{ fin_strip(rbox, angle, y, foot_width) }, offset(row, float(thickness))));
            append(thin, intersection(Polygons{ fin_strip(rbox, angle, y, tine_width) }, row));
            tines_along.emplace_back(fin_strip(rbox, angle, y, tine_width));
            any = true;
        };
        for (int k = 0; k < rows; ++ k) {
            const coord_t y = rb.min.y() + coord_t((k + 0.5) * double(rb.size().y()) / rows);
            Polygons row;
            for (const ExPolygon &piece : intersection_ex(Polygons{ fin_strip(rbox, angle, y, thickness) }, allowed))
                if (piece.area() >= min_piece)
                    append(row, to_polygons(piece));
            if (! row.empty())
                row = opening(diff(row, clear), min_width);
            if (! row.empty())
                add_row(y, row);
        }
        // An island with nothing on it still gets one fin through its center, standing on itself to the bed.
        if (! any) {
            Point c = island.contour.centroid();
            c.rotate(-angle);
            const Polygons row = intersection(Polygons{ fin_strip(rbox, angle, c.y(), thickness) }, island_p);
            if (! row.empty())
                add_row(c.y(), row);
        }
        append(walls, rails);

        // Scaffold: cross fins along the long axis, bracing the rows. Tines stay on the rows.
        if (cross_pitch > 0) {
            const double cross_angle = angle + 0.5 * M_PI;
            Polygon cframe = island.contour;
            cframe.rotate(-cross_angle);
            const BoundingBox crb = cframe.bounding_box();
            BoundingBox crbox = crb;
            crbox.offset(foot_width);
            const int crows = std::max(1, int(std::lround(double(crb.size().y()) / double(cross_pitch))));
            for (int k = 0; k < crows; ++ k) {
                const coord_t y = crb.min.y() + coord_t((k + 0.5) * double(crb.size().y()) / crows);
                append(walls, intersection(Polygons{ fin_strip(crbox, cross_angle, y, thickness) }, island_p));
                append(feet, intersection(Polygons{ fin_strip(crbox, cross_angle, y, foot_width) }, island_p));
            }
        }
    }
    walls        = union_(walls);
    feet         = union_(walls, feet);
    thin         = union_(thin);
    tines_along  = union_(tines_along);
    tines_across = union_(tines_across);
    const bool taper = thickness > 2 * spacing && ! thin.empty();
    const std::vector<bool> tine_row = fin_tine_layers(object);

    const int      interface_layers = config.support_fin_interface_layers.value;
    // Orient line-based interfaces across the fins so they can bridge from fin to fin.
    // Keep the normal interface filament, pattern, spacing and ironing settings.
    support_params.support_style          = smsGrid; // grid style prints interfaces at interface_angle
    support_params.interface_angle        = float(interface_dir + 0.5 * M_PI);
    const coordf_t top_gap_below    = config.support_top_z_distance.value;

    std::vector<Polygons> shaped(num_layers);
    std::vector<Polygons> interfaces(num_layers);
    std::vector<Polygons> tine_contacts(num_layers);
    tbb::parallel_for(tbb::blocked_range<size_t>(0, num_layers), [&](const tbb::blocked_range<size_t> &range) {
        for (size_t i = range.begin(); i < range.end(); ++ i) {
            if (regions[i].empty())
                continue;
            const Layer &layer = *layers[i];
            const bool   foot  = layer.bottom_z() < kBaseHeight - EPSILON;
            Polygons     out   = opening(intersection(regions[i], foot ? feet : walls), min_width);
            // A speck smaller than one loop square, e.g. a fin tip clipped where the object meets the bed, holds nothing.
            out.erase(std::remove_if(out.begin(), out.end(), [&](const Polygon &p) { return std::abs(p.area()) < double(2 * spacing) * double(2 * spacing); }), out.end());

            // 4. Within kTipHeight under the object, the fin narrows to one loop.
            if (taper && ! foot && ! out.empty()) {
                Polygons tip;
                for (size_t j = i + 1; j < num_layers && layers[j]->bottom_z() < layer.print_z + top_gap + kTipHeight + EPSILON; ++ j)
                    if (layers[j]->bottom_z() > layer.print_z + top_gap - EPSILON)
                        append(tip, outlines[j]);
                if (! tip.empty()) {
                    tip = union_(tip);
                    out = opening(union_(diff(out, tip), intersection(intersection(out, tip), thin)), min_width);
                }
            }

            // 5. Tines. The first object layer, which sits on the bed, never gets them.
            if (! out.empty() && tine_row[i]) {
                const float    gap           = float(scale_(support_params.gap_xy));
                const Polygons object_slices = to_polygons(layer.lslices);
                // Ring from `depth` inside the object surface out to just past the XY gap.
                const Polygons ring      = diff(offset(object_slices, gap + float(spacing)), offset(object_slices, - depth));
                const Polygons fin_touch = offset(out, float(SCALED_EPSILON));
                const Polygons near_fin  = offset(out, gap + depth + float(spacing));
                Polygons       tines;
                for (const Polygons *strips : { &tines_along, &tines_across }) {
                    Polygons found = intersection(intersection(*strips, ring), near_fin);
                    found.erase(std::remove_if(found.begin(), found.end(), [&](const Polygon &tine) {
                        // A tine must bridge from its fin into the object; one that misses either end
                        // (e.g. a short piece cut off at a corner of the ring) prints in mid-air.
                        if (intersection(Polygons{ tine }, fin_touch).empty() || intersection(Polygons{ tine }, object_slices).empty())
                            return true;
                        // A strip running along the object surface would leave a long piece inside the ring:
                        // a welded seam, not a tine.
                        return std::abs(tine.area()) / double(tine_width) > 3. * (gap + depth + float(spacing));
                    }), found.end());
                    append(tines, std::move(found));
                }
                if (!tines.empty()) {
                    tines = union_(tines);
                    out = union_(out, tines);
                    tine_contacts[i] = std::move(tines);
                }
            }

            // 6. Interface: the part of the fin region within interface_layers layers under the object
            // (below the top Z gap) becomes an interface deck; the fins stop under it.
            if (interface_layers > 0) {
                Polygons over;
                for (size_t j = i + 1; j < num_layers && layers[j]->bottom_z() < layer.print_z + top_gap_below + interface_layers * layer.height - EPSILON; ++ j)
                    append(over, outlines[j]);
                Polygons deck = opening(intersection(regions[i], over), min_width);
                if (! deck.empty()) {
                    out           = diff(out, deck);
                    interfaces[i] = std::move(deck);
                }
            }
            shaped[i] = std::move(out);
        }
    });
    // 7. Judge, when a fallback can take what the fins do not hold.
    if (reserved_columns != nullptr) {
        const Polygons uncovered = judge_fins(object, outlines, overhangs, shaped, top_gap, min_width);
        // The fins give way to the fallback there.
        const Polygons give_way  = offset(uncovered, 0.5f * float(spacing));
        // Give supplemental supports complete vertical columns wherever a real overhang is not held.
        // Reserving whole columns keeps both generators from cutting the other one's load-bearing path to the bed.
        reserved_columns->clear();
        for (size_t i = 0; i < num_layers; ++i) {
            if (! give_way.empty()) {
                shaped[i]        = diff(shaped[i], give_way);
                interfaces[i]    = diff(interfaces[i], give_way);
                tine_contacts[i] = diff(tine_contacts[i], give_way);
            }
            if (!shaped[i].empty() || !interfaces[i].empty()) {
                append(*reserved_columns, regions[i]);
                append(*reserved_columns, shaped[i]);
            }
        }
        *reserved_columns = diff(offset(union_(*reserved_columns), 0.5f * float(spacing)), give_way);
        reserve_holes();
    }
    regions.clear();

    // A fin much taller than it is long is a free-standing blade that can wobble or snap mid-print,
    // e.g. under a small feature high above the bed. Measure each fin above the foot; its length is
    // its area over its thickness, which holds for rows, rails and rim pieces alike.
    bool slender = false;
    {
        size_t first = 0;
        while (first < num_layers && (shaped[first].empty() || layers[first]->bottom_z() < kBaseHeight - EPSILON))
            ++ first;
        if (first < num_layers)
            for (const ExPolygon &fin : union_ex(shaped[first])) {
                const double   length = fin.area() / double(thickness);
                const Polygons column = offset(fin, float(spacing));
                size_t         top    = first;
                while (top + 1 < num_layers && ! intersection(shaped[top + 1], column).empty())
                    ++ top;
                if (layers[top]->print_z > kSlenderRatio * unscale<double>(length))
                    slender = true;
            }
    }

    // Hand the fins to the common support pipeline as base layers, one per object layer.
    SupportGeneratorLayerStorage layer_storage;
    SupportGeneratorLayersPtr    base_layers;
    SupportGeneratorLayersPtr    interface_layers_out;
    for (size_t i = 0; i < num_layers; ++ i)
        if (! interfaces[i].empty()) {
            SupportGeneratorLayer &support_layer = layer_storage.allocate_unguarded(SupporLayerType::TopInterface);
            support_layer.print_z  = layers[i]->print_z;
            support_layer.bottom_z = layers[i]->bottom_z();
            support_layer.height   = layers[i]->height;
            support_layer.polygons = std::move(interfaces[i]);
            // The lowest deck layer spans the gaps between fins.
            support_layer.bridging = i == 0 || interface_layers_out.empty() || interface_layers_out.back()->print_z < layers[i - 1]->print_z - EPSILON;
            interface_layers_out.push_back(&support_layer);
        }
    for (size_t i = 0; i < num_layers; ++ i)
        if (! shaped[i].empty()) {
            SupportGeneratorLayer &support_layer = layer_storage.allocate_unguarded(SupporLayerType::Base);
            support_layer.print_z  = layers[i]->print_z;
            support_layer.bottom_z = layers[i]->bottom_z();
            support_layer.height   = layers[i]->height;
            support_layer.polygons = std::move(shaped[i]);
            if (!tine_contacts[i].empty())
                support_layer.fin_contact_polygons = std::make_unique<Polygons>(std::move(tine_contacts[i]));
            base_layers.push_back(&support_layer);
        }
    if (base_layers.empty() && interface_layers_out.empty())
        return false;

    const SupportGeneratorLayersPtr empty;
    generate_support_layers(object, empty, empty, empty, base_layers, interface_layers_out, empty);
    generate_support_toolpaths(object.support_layers(), config, support_params, slicing_params, empty, empty, empty, base_layers, interface_layers_out, empty);

    BOOST_LOG_TRIVIAL(info) << "Fin support - end";
    return slender;
}

} // namespace Slic3r
