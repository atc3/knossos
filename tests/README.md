# tests

KNOSSOS has no test harness, and CMake does not compile anything in this directory
(`CMakeLists.txt` globs `annotation/ mesh/ scriptengine/ segmentation/ slicer/ skeleton/
tinyply/ widgets/` only). These are standalone programs for the parts that are pure
computation and worth pinning down.

## distancetransform_test

Exercises `segmentation/distancetransform.h`, the signed Euclidean distance transform and
distance-field blend behind shape interpolation. Checks the sign convention, that
anisotropic voxel spacing is measured in nanometres rather than pixels, that interpolating
between two discs tracks the expected radius, that the key slices reproduce exactly at
t=0 and t=1, and that centroid alignment keeps a laterally drifting shape from pinching or
vanishing.

```bash
c++ -std=c++17 -O2 -I .. -o /tmp/dt_test distancetransform_test.cpp && /tmp/dt_test
```

Exits non-zero on failure.

## sislice_test

Exercises `segmentation/sislice.h`, the 2D key-slice mask. Mostly index arithmetic: global
coordinate ↔ mask index round-trips at magnification 1 and 4, that growing the bounding box
downward shifts the origin so painted voxels keep reading back at the same global
coordinate, that the origin stays on the magnification lattice, erase accounting, and
`shrinkToFit`.

It also covers `siReachedBlocks`, which decides which blocks of the volume a mask actually
reaches into so the write walk can skip the rest. That matters for speed — a block nothing
is written to never enters the loader's cache, so every subsequent slice fetched it again,
which for a wide object was gigabytes of downloads to write nothing — but a block wrongly
cleared means part of the object is silently never written. So it is checked against a
brute-force reference that maps every set voxel to its block, over 400 randomised masks
with magnifications and grid origins that deliberately do not line up, plus an assertion
that it never clears a block holding part of the mask.

```bash
c++ -std=c++17 -O2 -I .. -o /tmp/sislice_test sislice_test.cpp && /tmp/sislice_test
```

## viewportlayout_test

Exercises `widgets/viewportlayoutgeometry.h`, the placement arithmetic behind the named
viewport arrangements. Checks that the reference unit fits an arrangement into the window
(whichever of width and height binds), that nothing escapes the window at any aspect ratio,
that the built-in shapes are what they claim, and that capturing the current arrangement
and re-applying it reproduces it — including preserving empty space the user deliberately
left, which an earlier version stretched away.

```bash
c++ -std=c++17 -O2 -I .. -o /tmp/vplayout_test viewportlayout_test.cpp && /tmp/vplayout_test
```

## undohistory_test

Exercises `widgets/historytimeline.h`, the arithmetic behind the History window and the
undo budget: that a row in the history list maps to the right number of undo/redo steps
(with and without redoable states above the current one), that clicking a row lands on
that row, and that budget eviction respects both the entry-count and total-size caps while
never dropping the last remaining entry. This caught a sign error in the redo mapping that
would have jumped to the wrong state whenever more than one redo was available.

```bash
c++ -std=c++17 -O2 -I .. -o /tmp/undohistory_test undohistory_test.cpp && /tmp/undohistory_test
```

## datasetmaxid_test

Pins the `MaxId` key a `.k.toml` `[[Layer]]` table may declare (`Dataset::maxId`, read in
`dataset.cpp`). Checks that an absent key reads as 0 — the sentinel for "undeclared", which
is what stops a plain image layer from claiming a max id — and that a declared one reads
back exactly, including a value past 32 bits, since subobject ids are 64 bit and truncating
one would silently hand out ids that are already taken.

Needs toml11, which the build fetches; the path below is where CMake's FetchContent puts it.

```bash
c++ -std=c++17 -O2 -I ../../knossos-build/_deps/toml11-src/single_include \
    -o /tmp/datasetmaxid_test datasetmaxid_test.cpp && /tmp/datasetmaxid_test
```

## holefill_test

Exercises `segmentation/holefill.h`, the connectivity behind "close an outline and the
middle fills in". Checks that an open outline encloses nothing and a closed one encloses
exactly its interior, that several holes in one plane are all found, that the region border
counts as the outside (which is why the caller grows its region until the object stops
touching it), and that degenerate input is refused rather than read out of bounds.

The case worth having a test for is the diagonal: a round brush dragged at 45° leaves
voxels touching only at their corners, which the eye reads as a closed wall. The escape
walk is therefore 4-connected — an 8-connected one slips between them and reports a closed
ring as open.

```bash
c++ -std=c++17 -O2 -I .. -o /tmp/holefill_test holefill_test.cpp && /tmp/holefill_test
```

## inventoryaccumulator_test

Exercises `segmentation/inventoryaccumulator.h`, the tally behind the object inventory: the
Z-order sweep, the block-to-magnification-1 coordinate arithmetic, and the choice of which
voxel to remember per object.

Two properties are worth having pinned. The first is that the stored position is a real
voxel of its object: the mean of a bent shape sits outside it, so a centroid would send
"next object" to empty neuropil beside a vessel rather than into it — there is a test with
an L-shaped object asserting that its mean is *not* on it while the stored position is. The
second is that the answer does not depend on the order blocks arrive in, since they arrive
over minutes, out of order, and across restarts; the test shuffles them and requires every
object to come out identical, and separately checks that a scan resumed halfway from the
cache matches one that never stopped.

The block-count checks also pin the real dataset sizes the feature was designed around
(1344 blocks at 8× for the largest volume, 880 at magnification 1 for a crop), because
those numbers are what the scan budget is set against.

```bash
c++ -std=c++17 -O2 -I .. -o /tmp/invacc_test inventoryaccumulator_test.cpp && /tmp/invacc_test
```

## inventoryfilter_test

Exercises `widgets/tools/inventoryfilter.h`, which decides what the Inventory tab shows and
where the next/previous keys go.

The guarantee the feature rests on is that pressing "next" twenty times shows twenty
different objects, once each, while a sweep is still appending to the list underneath. So
the central test is a randomised one: 200 scans built batch by batch must produce exactly
what rebuilding the row list from scratch would, since any divergence means the key silently
skips objects. It also pins that the row list stays strictly increasing, that a batch
contributing nothing reports nothing (so no row-insertion signal is emitted), that raising
the minimum size mid-walk resumes at the next surviving object instead of jumping back to
the top, and that the walk caps at both ends rather than wrapping — at this list length a
silent wrap is indistinguishable from the key having done nothing.

```bash
c++ -std=c++17 -O2 -I .. -o /tmp/invfilter_test inventoryfilter_test.cpp && /tmp/invfilter_test
```
