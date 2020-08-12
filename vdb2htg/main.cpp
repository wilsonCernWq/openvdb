// Copyright 2019-2020 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include <fstream>
#include <iostream>
#include <deque>

#include "openvdb/openvdb.h"

// Platform: Linux
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

// ----------------------------------------------------------------------------
// File I/O
// ----------------------------------------------------------------------------
struct FileMap
{
    enum IO_TYPE
    {
        BINARY_WRITE,
        BINARY_READ
    } type;
    char *map = (char *)MAP_FAILED;
    int fd = -1;
    uint64_t p = 0;
    uint64_t map_size = 0;
    uint64_t file_size = 0;
};

inline FileMap filemap_write_create(const std::string &filename,
                                    uint64_t requested_size)
{
    /* Open a file for writing.
     *  - Creating the file if it doesn't exist.
     *  - Truncating it to 0 size if it already exists. (not really needed)
     *
     * Note: "O_WRONLY" mode is not sufficient when mmaping.
     */

    FileMap ret;
    ret.type = FileMap::BINARY_WRITE;

    int &fd = ret.fd;
    char *&map = ret.map;

    fd = open(filename.c_str(), O_RDWR | O_CREAT | O_TRUNC, (mode_t)0600);

    if (fd == -1)
    {
        perror("Error opening file for writing");
        throw std::runtime_error("Termination caused by I/O errors");
    }

    // Stretch the file size to the size of the (mmapped) array of char

    if (lseek(fd, requested_size - 1, SEEK_SET) == -1)
    {
        close(fd);
        perror("Error calling lseek() to 'stretch' the file");
        throw std::runtime_error("Termination caused by I/O errors");
    }

    /* Something needs to be written at the end of the file to
     * have the file actually have the new size.
     * Just writing an empty string at the current file position will do.
     *
     * Note:
     *  - The current position in the file is at the end of the stretched
     *    file due to the call to lseek().
     *  - An empty string is actually a single '\0' character, so a zero-byte
     *    will be written at the last byte of the file.
     */

    if (write(fd, "", 1) == -1)
    {
        close(fd);
        perror("Error writing last byte of the file");
        throw std::runtime_error("Termination caused by I/O errors");
    }

    // Now the file is ready to be mmapped.
    map = (char *)mmap(
        0, requested_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (map == MAP_FAILED)
    {
        close(fd);
        perror("Error mmapping the file");
        throw std::runtime_error("Termination caused by I/O errors");
    }

    ret.map_size = requested_size;
    ret.file_size = requested_size;
    return ret;
}

inline FileMap filemap_read_create(const std::string &filename,
                                   uint64_t requested_size = 0)
{
    FileMap ret;
    ret.type = FileMap::BINARY_READ;

    int &fd = ret.fd;
    char *&map = ret.map;

    fd = open(filename.c_str(), O_RDONLY, (mode_t)0600);

    if (fd == -1)
    {
        perror("Error opening file for writing");
        throw std::runtime_error("Termination caused by I/O errors");
    }

    struct stat fileInfo = {0};

    if (fstat(fd, &fileInfo) == -1)
    {
        perror("Error getting the file size");
        throw std::runtime_error("Termination caused by I/O errors");
    }

    if (fileInfo.st_size == 0)
    {
        throw std::runtime_error("Error: File is empty, nothing to do");
    }

    printf("File size is %ji\n", (intmax_t)fileInfo.st_size);

    map = (char *)mmap(0, fileInfo.st_size, PROT_READ, MAP_SHARED, fd, 0);
    if (map == MAP_FAILED)
    {
        close(fd);
        perror("Error mmapping the file");
        throw std::runtime_error("Termination caused by I/O errors");
    }

    assert(requested_size <= fileInfo.st_size);
    ret.map_size = requested_size == 0 ? fileInfo.st_size : requested_size;
    ret.file_size = fileInfo.st_size;
    return ret;
}

inline void filemap_write(FileMap &file,
                          const void *data,
                          const uint64_t bytes)
{
    assert(file.type == FileMap::BINARY_WRITE);
    assert(bytes <= file.map_size);

    printf("Writing %zu bytes\n", bytes);

    // Write data to in-core memory
    const char *text = (const char *)data;
    for (uint64_t i = 0; i < bytes; i++)
    {
        file.map[file.p + i] = text[i];
    }

    file.p += bytes;
}

inline void filemap_read(FileMap &file, void *data, const uint64_t bytes)
{
    assert(file.type == FileMap::BINARY_READ);
    assert(bytes <= file.map_size);

    printf("Read %zu bytes\n", bytes);

    char *text = (char *)data;
    for (uint64_t i = 0; i < bytes; i++)
    {
        text[i] = file.map[file.p + i];
    }

    file.p += bytes;
}

inline void filemap_close(FileMap &file)
{
    // Don't forget to free the mmapped memory
    if (munmap(file.map, file.map_size) == -1)
    {
        close(file.fd);
        perror("Error un-mmapping the file");
        throw std::runtime_error("Termination caused by I/O errors");
    }

    // Un-mmaping doesn't close the file, so we still need to do that.
    close(file.fd);
}

// ----------------------------------------------------------------------------
// Builder Implementation
// ----------------------------------------------------------------------------
struct vec3f
{
    float x, y, z;
};
struct range1f
{
    float lower, upper;
};

int main(int argc, char *argv[])
{
    openvdb::initialize();
    openvdb::logging::initialize(argc, argv);

    // std::string data_name = "smoke";
    // std::string data_name = "smoke2";
    // std::string data_name = "bunny_cloud";
    // std::string data_name = "explosion";
    // std::string name = "/home/qadwu/Data/openvdb/" + data_name + ".vdb";

    // std::string data_name = "wdas_cloud_sixteenth";
    // std::string data_name = "wdas_cloud_eighth";
    // std::string data_name = "wdas_cloud_quarter";
    // std::string data_name = "wdas_cloud_half";
    std::string data_name = "wdas_cloud";
    std::string name = "/home/qadwu/Data/openvdb/wdas_cloud/" + data_name + ".vdb";

    // Create a VDB file object.
    printf("working on %s\n", name.c_str());
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
            openvdb::GridBase::Ptr _g = file.readGrid(nameIter.gridName());
            printf("skipping grid %s (%s)\n", nameIter.gridName().c_str(), _g->type().c_str());
        }
    }
    file.close();

    // From the example above, "LevelSetSphere" is known to be a FloatGrid,
    // so cast the generic grid pointer to a FloatGrid pointer.
    openvdb::FloatGrid::Ptr grid = openvdb::gridPtrCast<openvdb::FloatGrid>(baseGrid);

    // Return the number of active voxels. More...
    auto g_count = grid->activeVoxelCount();
    std::cout << "active voxel count " << g_count << std::endl;

    // Return the axis-aligned bounding box of all active voxels. More...
    auto g_bbox = grid->evalActiveVoxelBoundingBox();
    std::cout << "actual bounding box " << g_bbox << std::endl;

    // Return the dimensions of the axis-aligned bounding box of all active voxels. More...
    auto g_dims = grid->evalActiveVoxelDim();
    std::cout << "shifted bounding box " << g_dims << std::endl;
    std::cout << "total voxel count " << (size_t)g_dims.x() * g_dims.y() * g_dims.z() << std::endl;
    std::cout << "sparseness " << (float)g_count / ((size_t)g_dims.x() * g_dims.y() * g_dims.z()) << std::endl;

    // Return the number of bytes of memory used by this grid.
    auto g_ram = grid->memUsage();
    std::cout << "mem usage " << g_ram << "(B)" << std::endl;

    // Visit and update all of the grid's active values, which correspond to voxels on the narrow band.
    vec3f actualBounds;
    actualBounds.x = g_dims.x() + 1.f;
    actualBounds.y = g_dims.y() + 1.f;
    actualBounds.z = g_dims.z() + 1.f;

    const uint64_t numValues = grid->activeVoxelCount();
    const uint64_t dataSize = sizeof(float) * 7 * numValues;
    std::cout << "numValues " << numValues << std::endl;
    std::cout << "dataSize " << dataSize << std::endl;

    auto mapper = filemap_write_create(data_name + ".vox", dataSize + 2 * sizeof(uint64_t) + sizeof(vec3f));
    filemap_write(mapper, &dataSize, sizeof(uint64_t));
    filemap_write(mapper, &numValues, sizeof(uint64_t));
    filemap_write(mapper, &actualBounds, sizeof(vec3f));

    vec3f *lowers = (vec3f *)&mapper.map[mapper.p];
    float *widths = (float *)&mapper.map[mapper.p + numValues * 3 * sizeof(float)];
    float *values = (float *)&mapper.map[mapper.p + numValues * 4 * sizeof(float)];
    range1f *ranges = (range1f *)&mapper.map[mapper.p + numValues * 5 * sizeof(float)];

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
            lowers[index] = coord;
            widths[index] = width;
            values[index] = f;
            ranges[index] = range;
            ++index;
        }
        else
        {
            // bounding boxes in the index space
            openvdb::CoordBBox bbox;
            iter.getBoundingBox(bbox);
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
                        lowers[index] = coord;
                        widths[index] = width;
                        values[index] = f;
                        ranges[index] = range;
                        ++index;
                    }
                }
            }
        }
    }

    assert(index == numValues);

    filemap_close(mapper);

    return 0;
}
