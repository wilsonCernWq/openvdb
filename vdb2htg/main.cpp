// Copyright 2019-2020 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include <fstream>
#include <iostream>
#include <deque>

#define HTG_STANDALONE
#include "/home/qadwu/Work/intel/openvkl/repo/openvkl/drivers/ispc/volume/htg/HtgNode.h"

#include "openvdb/openvdb.h"

// #include "cereal/types/unordered_map.hpp"
// #include "cereal/types/memory.hpp"
// #include "cereal/archives/binary.hpp"

int main(int argc, char *argv[])
{
    openvdb::initialize();
    openvdb::logging::initialize(argc, argv);

    // std::string data_name = "smoke";
    // std::string data_name = "smoke2";
    std::string data_name = "bunny_cloud";
    // std::string data_name = "explosion";
    std::string name = "/home/qadwu/Data/openvdb/" + data_name + ".vdb";

    // Create a VDB file object.
    openvdb::io::File file(name);

    // Open the file.  This reads the file header, but not any grids.
    file.open();

    // Loop over all grids in the file and retrieve a shared pointer
    // to the one named "density".
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
    box3f actualBounds;
    actualBounds.lower.x = 0.f;
    actualBounds.lower.y = 0.f;
    actualBounds.lower.z = 0.f;
    actualBounds.upper.x = g_dims.x() + 1.f;
    actualBounds.upper.y = g_dims.y() + 1.f;
    actualBounds.upper.z = g_dims.z() + 1.f;

    auto numValues = grid->activeVoxelCount();

    openvkl::ispc_driver::HtgVoxels voxels(numValues, sizeof(float));

    size_t index = 0;
    for (openvdb::FloatGrid::ValueOnIter iter = grid->beginValueOn(); iter; ++iter)
    {
        float f = iter.getValue();

        vec3f coord;
        float width;
        range1f range;

        range.lower = f;
        range.upper = f;

        if (iter.isVoxelValue())
        {
            // indices in the index space
            auto _coord = iter.getCoord();
            coord.x = _coord.x() - g_bbox.min().x();
            coord.y = _coord.y() - g_bbox.min().y();
            coord.z = _coord.z() - g_bbox.min().z();
            width = 1.f;
            voxels.setVoxel<float>(index++, coord, width, f, range);
        }
        else
        {
            // bounding boxes in the index space
            openvdb::CoordBBox bbox;
            iter.getBoundingBox(bbox);
            std::cout << bbox << std::endl;
            // OpenVDB tile should be cubic according to its data structure?
            for (size_t z = 0; z < bbox.dim().z(); ++z)
            {
                for (size_t y = 0; y < bbox.dim().y(); ++y)
                {
                    for (size_t x = 0; x < bbox.dim().x(); ++x)
                    {
                        coord.x = bbox.min().x() + (float)x - g_bbox.min().x();
                        coord.y = bbox.min().y() + (float)y - g_bbox.min().y();
                        coord.z = bbox.min().z() + (float)z - g_bbox.min().z();
                        width = 1.f;
                        voxels.setVoxel<float>(index++, coord, width, f, range);
                    }
                }
            }
            voxels.setVoxel<float>(index++, coord, -1.f, f, range);
        }
    }

    // filter voxels
    voxels.filter();

    openvkl::ispc_driver::HtgBuilder<float>
        builder(actualBounds.upper, voxels, voxels.size());
    builder.build();
    builder.print();

    using namespace openvkl::ispc_driver;

    const range1f range = getValueRange(builder.data[0], 0);
    const uint64_t numOfNodes = builder.data.size();
    const uint64_t dataSize = sizeof(HtgNode) * numOfNodes + 2 * sizeof(box3f) + sizeof(size_t);

    auto mapper = filemap_write_create(data_name + ".stm", dataSize);

    // file format
    filemap_write(mapper, &numOfNodes, sizeof(uint64_t));
    filemap_write(mapper, &builder.actualBounds, sizeof(vec3f));
    filemap_write(mapper, &builder.extendBounds, sizeof(float));
    filemap_write(mapper, &range.lower, sizeof(float));
    filemap_write(mapper, &range.upper, sizeof(float));
    filemap_write(mapper, builder.data.data(), sizeof(HtgNode) * builder.data.size());
    filemap_close(mapper);

    return 0;
}
