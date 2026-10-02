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

## planarwrite_test

Exercises `segmentation/planarwrite.h`, which works out the blocks one slice of an
interpolation has to be written into and where in each. It is integer arithmetic over three
axes in an order that changes with the viewing plane, a magnification lattice the block grid
need not line up with, and a movement area whose upper bound is exclusive — and a mistake
in it does not crash or warn, it quietly leaves part of the object unwritten.

The central check is an equivalence one. Leaving out blocks the mask does not reach is a
large speed-up, because a block nothing is written to never enters the loader's cache and
so gets fetched again for every later slice. To show that the fast plan writes the same
result as the exhaustive one, the test runs both through a simulated volume — stepping the
lattice the way `processRegion` does, from the block's origin and capped to the region — and
requires the two to come out identical, over 300 randomised cases covering magnifications
1/2/4, block shapes that are not powers of two, mask origins off the block grid, and all
three slice orientations.

Density is what decides whether a mistake here is visible, so the randomised shapes include
blobs (whole blocks fall empty, which is what the skip is for), salt and pepper (a *single*
isolated voxel in a block, which catches any test for "reached" stricter than "at least
one"), and a solid fill as the control where nothing may be skipped. There is also an
explicit lone-voxel case at each magnification.

Worth knowing if you change this: the test was checked by mutation, and the first two
mutations tried — shortening the scan run by one, and requiring two set voxels — were *not*
caught, because the shapes in the original version were too dense to expose them. The sparse
shapes above were added for that reason. Mutations now caught include an off-by-one in the
block index, not examining the last mask row, requiring a run longer than one voxel, and
treating the movement area's maximum as inclusive.

```bash
c++ -std=c++17 -O2 -I .. -o /tmp/planarwrite_test planarwrite_test.cpp && /tmp/planarwrite_test
```

## precomputed_test

Exercises `segmentation/precomputed.h`, which reads a Neuroglancer precomputed
segmentation — the format a dataset declares with `ServerFormat = 'precomputed'`, and which
KNOSSOS previously loaded as an empty layer because every request for a KNOSSOS-style cube
path missed.

Two things in it are worth testing on their own. The `compressed_segmentation` encoding is
bit-packed, and an off-by-one in the packing yields a plausible-looking volume of wrong
labels rather than a failure — so the decoder is checked by round trip against an encoder
written in the test from the spec, over 28 volumes covering every index width the format
allows (including the zero-bit case, where a block of one label costs 8 bytes and no bits
per voxel) and extents that are not multiples of the block size, so blocks get clipped at
the volume's far face. And chunk addressing in a sharded store needs a *compressed* Morton
code, where each axis contributes only as many bits as its grid needs and the others close
up once one is exhausted: a plain three-way interleave is wrong for any grid that is not a
cube, which is every real grid.

The shard filename cases come from a real store rather than from reading the spec: 8 and 5
shard bits were observed to give two lowercase hex digits (`0f.shard`, `01.shard`, and
`10.shard`), 2 and 0 bits to give one (`0.shard`). Shards holding no chunks are simply
absent, so a 404 there is ordinary.

The decoder was also validated against a real sharded dataset — one 121×54×77 chunk at the
coarsest level, 2272 distinct labels, 99.5% background — by decoding it independently in
Python from the spec and requiring the two to agree byte for byte. That data is somebody
else's and is not in the repository, so what is kept here is the synthetic round trip.

```bash
c++ -std=c++17 -O2 -I .. -o /tmp/precomputed_test precomputed_test.cpp && /tmp/precomputed_test
```
