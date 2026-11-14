# Support Advanced

- [Z distance](#z-distance)
- [Support wall loops](#support-wall-loops)
- [Base Pattern](#base-pattern)
  - [Base pattern spacing](#base-pattern-spacing)
- [Pattern angle](#pattern-angle)
- [Interface layers](#interface-layers)
- [Interface pattern](#interface-pattern)
- [Interface spacing](#interface-spacing)
- [Normal support expansion](#normal-support-expansion)
- [Support/object XY distance](#supportobject-xy-distance)
- [Support/object first layer gap](#supportobject-first-layer-gap)
- [Don't support bridges](#dont-support-bridges)
- [Independent support layer height](#independent-support-layer-height)

## Z distance

The Z gap between support interface and object.

## Support wall loops

This setting specifies the count of support walls in the range of [0,2]. 0 means auto.

## Base Pattern

Line pattern for the base of the support.

### Base pattern spacing

Spacing between support lines.

## Pattern angle

Use this setting to rotate the support pattern on the horizontal plane.

## Interface layers

The number of interface layers.

## Interface pattern

The pattern used for the support interface.

## Interface min area

Minimum support contact area. Lower values generate more support contact surfaces.

## Interface spacing

Spacing of interface lines. Zero means solid interface.

## Fins

Fins are thin, solid walls printed under overhanging areas of the object, with small one-layer beads (tines) that fuse each fin to the object. The settings below control them. Source: `src/libslic3r/Support/SupportFins.*`.

### Supplemental supports

Add supports to overhangs the fins do not hold: more than 2 mm from a fin, and not bridged between two. Tree uses organic branches; Normal uses grid or snug supports. Regions are assigned to fins or supplemental supports to avoid overlapping structures. The overhang threshold and build-plate-only settings apply to supplemental supports. Both use the same support filament, interface filament, interface layers, pattern and spacing. Off preserves fins-only behavior.

### Fin spacing

Distance between neighboring rows of fins. Rows run across the long axis of each area under the object. A row shorter than 4 mm is left out.

### Fin thickness

Wall thickness of each fin. Fins are printed solid with loops of two lines each, so the thickness is rounded to whole loops.

### Tine spacing

Vertical distance between tines. A tine is a one-layer bead that fuses a fin to the object so the object cannot lean away from it. Each one leaves a small mark. Tines only bond when the fins and the object use the same filament. Set to 0 to disable tines.

### Extra tines near the bed

Adds tines close together just above the fin foot, where the object is least stable, with gaps that widen going up until they reach the tine spacing.

### Tine depth

How far a tine reaches into the object. Deeper tines hold better but leave a larger mark after removal.

### Fin height

Fins are generated up to this height above the bed. Set to 0 to generate them as high as the object needs.

### Lean side only

Only generate fins on the side the object leans toward (where its center of mass lies beyond its bed contact), which is the side that holds it up. A balanced object gets fins on its larger side. Leave this off for objects that hang over the bed on several sides, such as a T shape, or the other sides are left without support.

### Cross fin spacing

Adds fins across the main fins every this distance, tying them into a scaffold. A braced fin is much stiffer than a single wall, which matters for tall fins. Set to 0 to disable.

## Normal support expansion

Expand (+) or shrink (-) the horizontal span of normal support.

## Support/object XY distance

XY separation between an object and its support.

## Support/object first layer gap

XY separation between an object and its support at the first layer.

## Don't support bridges

Don't support the whole bridge area which make support very large. Bridges can usually be printed directly without support if not very long.

## Independent support layer height

Support layer uses layer height independent with object layer. This is to support customizing z-gap and save print time. This option will be invalid when the prime tower is enabled.
