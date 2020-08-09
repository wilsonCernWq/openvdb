#include <iostream>
#include "openvdb/openvdb.h"

int main(int argc, char *argv[])
{

    std::cout << "hello" << std::endl;

    openvdb::initialize();
    openvdb::logging::initialize(argc, argv);

    // Create a VDB file object.
    //std::string name = "E:\\vdb\\v224\\sphere_points.vdb-1.0.0\\sphere_points.vdb";
    std::string name = "/home/qadwu/Data/openvdb/bunny_cloud.vdb";
    openvdb::io::File file(name);
    // Open the file.  This reads the file header, but not any grids.
    file.open();
    // Loop over all grids in the file and retrieve a shared pointer
    // to the one named "LevelSetSphere".  (This can also be done
    // more simply by calling file.readGrid("LevelSetSphere").)
    openvdb::GridBase::Ptr baseGrid;
    for (openvdb::io::File::NameIterator nameIter = file.beginName();
         nameIter != file.endName(); ++nameIter)
    {
        // Read in only the grid we are interested in.
        if (nameIter.gridName() == "density")
        {
            baseGrid = file.readGrid(nameIter.gridName());
            printf("loading grid %s (%s)\n", nameIter.gridName().c_str(), baseGrid->type().c_str());
        }
        else
        {
            std::cout << "skipping grid " << nameIter.gridName() << std::endl;
        }
    }
    file.close();

    // From the example above, "LevelSetSphere" is known to be a FloatGrid,
    // so cast the generic grid pointer to a FloatGrid pointer.
    openvdb::FloatGrid::Ptr grid = openvdb::gridPtrCast<openvdb::FloatGrid>(baseGrid);

    // Return the number of active voxels. More...
    auto g_count = grid->activeVoxelCount();
    std::cout << "activeVoxelCount " << g_count << std::endl;

    // Return the axis-aligned bounding box of all active voxels. More...
    auto g_bbox = grid->evalActiveVoxelBoundingBox();
    std::cout << "evalActiveVoxelBoundingBox " << g_bbox << std::endl;

    // Return the dimensions of the axis-aligned bounding box of all active voxels. More...
    auto g_dims = grid->evalActiveVoxelDim();
    std::cout << "evalActiveVoxelDim " << g_dims << std::endl;
    std::cout << "evalActiveVoxelDim - count " << (size_t)g_dims.x() * g_dims.y() * g_dims.z() << std::endl;
    std::cout << "activeVoxelCount / count " << (float)g_count / ((size_t)g_dims.x() * g_dims.y() * g_dims.z()) << std::endl;

    // Return the number of bytes of memory used by this grid.
    auto g_ram = grid->memUsage();
    std::cout << "memUsage " << g_ram << "(Bytes)" << std::endl;

    // Visit and update all of the grid's active values, which correspond to
    // voxels on the narrow band.
    // for (openvdb::FloatGrid::ValueOnIter iter = grid->beginValueOn(); iter; ++iter)
    // {
    //     float dist = iter.getValue();
    //     if (iter.isVoxelValue())
    //     {
    //         printf("[value] %f ", dist);
    //         printf("VOXEL ");
    //         // indices in the index space
    //         std::cout << iter.getCoord() << std::endl;
    //     }
    //     else
    //     {
    //         printf("[value] %f ", dist);
    //         printf("TILE ");
    //         // bounding boxes in the index space
    //         openvdb::CoordBBox bbox;
    //         iter.getBoundingBox(bbox);
    //         std::cout << bbox << std::endl;
    //     }
    // }

    return 0;
}
