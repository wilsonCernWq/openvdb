#include <iostream>
#include "openvdb/openvdb.h"

int main(int argc, char *argv[]) {

    std::cout << "hello" << std::endl;
    
    openvdb::initialize();
    openvdb::logging::initialize(argc, argv);

    // Create a VDB file object.
    std::string name = "E:\\vdb\\v224\\sphere_points.vdb-1.0.0\\sphere_points.vdb";
    openvdb::io::File file(name);
    // Open the file.  This reads the file header, but not any grids.
    // file.open();
    // // Loop over all grids in the file and retrieve a shared pointer
    // // to the one named "LevelSetSphere".  (This can also be done
    // // more simply by calling file.readGrid("LevelSetSphere").)
    // openvdb::GridBase::Ptr baseGrid;
    // for (openvdb::io::File::NameIterator nameIter = file.beginName();
    //     nameIter != file.endName(); ++nameIter)
    // {
    //     // Read in only the grid we are interested in.
    //     // if (nameIter.gridName() == "LevelSetSphere") {
    //     //     baseGrid = file.readGrid(nameIter.gridName());
    //     // }
    //     // else {
    //         std::cout << "skipping grid " << nameIter.gridName() << std::endl;
    //     // }
    // }
    // file.close();

    // // From the example above, "LevelSetSphere" is known to be a FloatGrid,
    // // so cast the generic grid pointer to a FloatGrid pointer.
    // openvdb::FloatGrid::Ptr grid = openvdb::gridPtrCast<openvdb::FloatGrid>(baseGrid);
    // // Convert the level set sphere to a narrow-band fog volume, in which
    // // interior voxels have value 1, exterior voxels have value 0, and
    // // narrow-band voxels have values varying linearly from 0 to 1.
    // const float outside = grid->background();
    // const float width = 2.0 * outside;
    // // Visit and update all of the grid's active values, which correspond to
    // // voxels on the narrow band.
    // for (openvdb::FloatGrid::ValueOnIter iter = grid->beginValueOn(); iter; ++iter) {
    //     float dist = iter.getValue();
    //     iter.setValue((outside - dist) / width);
    // }
    // // Visit all of the grid's inactive tile and voxel values and update the values
    // // that correspond to the interior region.
    // for (openvdb::FloatGrid::ValueOffIter iter = grid->beginValueOff(); iter; ++iter) {
    //     if (iter.getValue() < 0.0) {
    //         iter.setValue(1.0);
    //         iter.setValueOff();
    //     }
    // }
    // // Set exterior voxels to 0.
    // openvdb::tools::changeBackground(grid->tree(), 0.0);

    return 0;
}