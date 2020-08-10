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

    std::string name = "/home/qadwu/Data/openvdb/bunny_cloud.vdb";
    // std::string name = "/home/qadwu/Data/openvdb/wdas_cloud/wdas_cloud_sixteenth.vdb";

    // Create a VDB file object.
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
    box3f actualBounds;
    actualBounds.lower.x = g_bbox.min().x();
    actualBounds.lower.y = g_bbox.min().y();
    actualBounds.lower.z = g_bbox.min().z();
    actualBounds.upper.x = g_bbox.max().x();
    actualBounds.upper.y = g_bbox.max().y();
    actualBounds.upper.z = g_bbox.max().z();

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
            coord.x = _coord.x();
            coord.y = _coord.y();
            coord.z = _coord.z();
            width = 1.f;
        }
        else
        {
            // bounding boxes in the index space
            openvdb::CoordBBox bbox;
            iter.getBoundingBox(bbox);
            // OpenVDB tile should be cubic according to its data structure?
            assert(bbox.dim().x() == bbox.dim().y());
            assert(bbox.dim().x() == bbox.dim().z());
            coord.x = bbox.min().x();
            coord.y = bbox.min().y();
            coord.z = bbox.min().z();
            width = bbox.dim().x();
        }

        voxels.setVoxel<float>(index++, coord, width, f, range);
    }

    // filter voxels
    voxels.filter();

    openvkl::ispc_driver::HtgBuilder<float>
        builder(actualBounds, voxels, voxels.size());
    builder.build();
    builder.print();

    using namespace openvkl::ispc_driver;

    const size_t numOfNodes = builder.data.size();
    const size_t dataSize = sizeof(HtgNode) * numOfNodes + 2 * sizeof(box3f) + sizeof(size_t);

    auto mapper = filemap_write_create("bunny_density.stm", dataSize);
    filemap_write(mapper, &builder.actualBounds, sizeof(builder.actualBounds));
    filemap_write(mapper, &builder.extendBounds, sizeof(builder.extendBounds));
    filemap_write(mapper, &numOfNodes, sizeof(numOfNodes));
    filemap_write(mapper, builder.data.data(), sizeof(HtgNode) * builder.data.size());
    filemap_close(mapper);

    auto reader = filemap_read_create("bunny_density.stm", dataSize);
    box3f _actualBounds, _extendBounds;
    size_t _numOfNodes;
    filemap_read(reader, &_actualBounds, sizeof(_actualBounds));
    filemap_read(reader, &_extendBounds, sizeof(_extendBounds));
    filemap_read(reader, &_numOfNodes, sizeof(_numOfNodes));
    filemap_close(reader);

    assert(_actualBounds.lower.x == builder.actualBounds.lower.x);
    assert(_actualBounds.lower.y == builder.actualBounds.lower.y);
    assert(_actualBounds.lower.z == builder.actualBounds.lower.z);
    assert(_actualBounds.upper.x == builder.actualBounds.upper.x);
    assert(_actualBounds.upper.y == builder.actualBounds.upper.y);
    assert(_actualBounds.upper.z == builder.actualBounds.upper.z);

    assert(_extendBounds.lower.x == builder.extendBounds.lower.x);
    assert(_extendBounds.lower.y == builder.extendBounds.lower.y);
    assert(_extendBounds.lower.z == builder.extendBounds.lower.z);
    assert(_extendBounds.upper.x == builder.extendBounds.upper.x);
    assert(_extendBounds.upper.y == builder.extendBounds.upper.y);
    assert(_extendBounds.upper.z == builder.extendBounds.upper.z);

    assert(_numOfNodes == numOfNodes);

    return 0;
}
