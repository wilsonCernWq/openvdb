// Copyright 2019-2020 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#if defined(__cplusplus)
// Platform: Linux
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <string.h>
#endif

// --------------------------------------------------------------------------------
//
// Streaming HTG Node Structure
//
// --------------------------------------------------------------------------------

#if defined(__cplusplus)
namespace openvkl
{
  namespace ispc_driver
  {
#endif

// node encoding  (MSB) | ... data ... | ... flag ... | (LSB)
// flag (4 bit) - indicating whether it is a leaf or an innner node
//              - indicating whether a internal node is evicted or not
//              - indicating whether a internal node has been requested recently
//              - researved (a value range node)
#define HTG_BIT_FLAGS 0xF
#define HTG_CLR_FLAGS (~HTG_BIT_FLAGS)
#define HTG_FLAG_IS_LEAF (1)
#define HTG_FLAG_EVICTED (1 << 1)
#define HTG_FLAG_REQUEST (1 << 2)
#define HTG_FLAG_V_RANGE (1 << 3)
// data (60bit) - internal node  : child desctiption
//              - leaf / evicted : the value of the node w/ 60 bit
// child desctiption - children offset
//                   - children mask
// children mask
#define HTG_OFF_CHILD_MASK 4
#define HTG_BIT_CHILD_MASK 0xFF
#define HTG_CLR_CHILD_MASK (~(HTG_BIT_CHILD_MASK << HTG_OFF_CHILD_MASK))
// children offset
#define HTG_OFF_CHILD_OFFSET 12
#define HTG_BIT_CHILD_OFFSET 0xFFFFFFFFFFFFF
#define HTG_CLR_CHILD_OFFSET (~(HTG_BIT_CHILD_OFFSET << HTG_OFF_CHILD_OFFSET))

    struct HtgNode
    {
      uint64_t _encoded;
      range1f _range;
#if defined(__cplusplus)
      HtgNode() = default;
      HtgNode(uint64_t offset);
      void set(uint64_t);
#endif
    };

    inline __varying uint8_t computeHtgNodeChildIndex(
        const __varying uint8_t mask, const __varying uint8_t octant)
    {
      return octant;
    }

#define __htg_access_functions(self, data)                                     \
                                                                               \
  inline __varying bool isLeaf(const self)                                     \
  {                                                                            \
    return (data(_encoded) & HTG_FLAG_IS_LEAF);                                \
  }                                                                            \
                                                                               \
  inline void setAsLeaf(self, __varying bool flag)                             \
  {                                                                            \
    data(_encoded) = flag ? (data(_encoded) | HTG_FLAG_IS_LEAF)                \
                          : (data(_encoded) & ~HTG_FLAG_IS_LEAF);              \
  }                                                                            \
                                                                               \
  inline __varying bool isEvicted(const self)                                  \
  {                                                                            \
    return (data(_encoded) & HTG_FLAG_EVICTED);                                \
  }                                                                            \
                                                                               \
  inline void setAsEvicted(self, __varying bool flag)                          \
  {                                                                            \
    data(_encoded) = flag ? (data(_encoded) | HTG_FLAG_EVICTED)                \
                          : (data(_encoded) & ~HTG_FLAG_EVICTED);              \
  }                                                                            \
                                                                               \
  inline __varying bool isValueRangeNode(const self)                           \
  {                                                                            \
    return (data(_encoded) & HTG_FLAG_V_RANGE);                                \
  }                                                                            \
                                                                               \
  inline void setAsValueRangeNode(self, __varying bool flag)                   \
  {                                                                            \
    data(_encoded) = flag ? (data(_encoded) | HTG_FLAG_V_RANGE)                \
                          : (data(_encoded) & ~HTG_FLAG_V_RANGE);              \
  }                                                                            \
                                                                               \
  /* a terminating node contains a 60-bit compressed value                     \
     a non terminating (i.e. internal) node contains a 8-bit mask              \
     and a 50-bit offset indicating its distance to its children */            \
  inline __varying bool isTerminatingNode(const self)                          \
  {                                                                            \
    return (data(_encoded) & HTG_FLAG_IS_LEAF) ||                              \
           (data(_encoded) & HTG_FLAG_EVICTED);                                \
  }                                                                            \
                                                                               \
  inline __varying uint8_t getChildMask(const self)                            \
  {                                                                            \
    assert(!isTerminatingNode(node));                                          \
    return (data(_encoded) >> HTG_OFF_CHILD_MASK) & HTG_BIT_CHILD_MASK;        \
  }                                                                            \
                                                                               \
  inline void setChildMask(self, __varying uint8_t mask)                       \
  {                                                                            \
    assert(!isTerminatingNode(node));                                          \
    const uint64_t shifted_mask = ((uint64_t)mask) << HTG_OFF_CHILD_MASK;      \
    data(_encoded) = (data(_encoded) & HTG_CLR_CHILD_MASK) | shifted_mask;     \
  }                                                                            \
                                                                               \
  inline __varying uint64_t getChildOffset(const self)                         \
  {                                                                            \
    assert(!isTerminatingNode(node));                                          \
    return data(_encoded) >> HTG_OFF_CHILD_OFFSET;                             \
  }                                                                            \
                                                                               \
  inline void setChildOffset(self, const __varying uint64_t &offset)           \
  {                                                                            \
    assert(!isTerminatingNode(node));                                          \
    const uint64_t shifted_offset = offset << HTG_OFF_CHILD_OFFSET;            \
    data(_encoded) = (data(_encoded) & HTG_CLR_CHILD_OFFSET) | shifted_offset; \
  }                                                                            \
                                                                               \
  inline __varying double getValue(self)                                       \
  {                                                                            \
    assert(isTerminatingNode(node)); /* give up mantissa bits */               \
    return decodeDouble(data(_encoded) & HTG_CLR_FLAGS);                       \
  }                                                                            \
                                                                               \
  inline void setValue(self, const __varying double &value)                    \
  {                                                                            \
    assert(isTerminatingNode(node)); /* save existing flags */                 \
    uint32_t flags = data(_encoded) & HTG_BIT_FLAGS;                           \
    data(_encoded) = (encodeDouble(value) & HTG_CLR_FLAGS) | flags;            \
  }

#define __htg_ptr_access(x) (node->x)
#define __htg_ref_access(x) (node.x)
#define __htg_ptr_addr(x) (node)
#define __htg_ref_addr(x) (&node)

//
// ISPC side functions
//
#if defined(ISPC)

    __htg_access_functions(uniform HtgNode *node, __htg_ptr_access);

    inline __varying range1f getValueRange(const uniform HtgNode *node)
    {
      // TODO convertion between 64bit addressing and 32bit addressing
      const __varying uint64_t offset = getChildOffset(node);
      const uniform HtgNode *children = node + offset + 8;
      return children->_range;
    }

#endif

//
// C++ side functions
//
#if defined(__cplusplus)

    __htg_access_functions(HtgNode *node, __htg_ptr_access);
    __htg_access_functions(HtgNode &node, __htg_ref_access);

    inline bool isInvalid(const HtgNode &node)
    {
      return node._encoded == uint64_t(-1);
    }

    inline range1f getValueRange(const HtgNode *node)
    {
      if (isValueRangeNode(node))
        return node->_range;
      else
      {
        const uint64_t o = getChildOffset(node);
        const HtgNode *n = node + o + 8;
        return n->_range;
      }
    }

    inline range1f getValueRange(const HtgNode &node)
    {
      if (isValueRangeNode(node))
        return node._range;
      else
      {
        const uint64_t o = getChildOffset(node);
        const HtgNode *n = &node + o + 8;
        return n->_range;
      }
    }

    inline void setValueRange(HtgNode *node, const range1f &value)
    {
      assert(isValueRangeNode(node));
      node->_range = value;
    }

    inline void setValueRange(HtgNode &node, const range1f &value)
    {
      assert(isValueRangeNode(node));
      node._range = value;
    }

    // --- other important member functions ---
    inline HtgNode::HtgNode(uint64_t offset)
    {
      setChildOffset(this, offset);
    }

    inline void HtgNode::set(uint64_t v)
    {
      _encoded = v;
    }

  } // namespace ispc_driver
} // namespace openvkl
#endif

#undef __htg_ptr_access
#undef __htg_ref_access
#undef __htg_ptr_addr
#undef __htg_ref_addr
#undef __htg_access_functions

// --------------------------------------------------------------------------------
//
// C++ Side Implementation
//
// --------------------------------------------------------------------------------
#if defined(__cplusplus)
namespace openvkl
{
  namespace ispc_driver
  {

    // ----------------------------------------------------------------------------
    // Builder Implementation
    // ----------------------------------------------------------------------------
    struct HtgVoxels
    {
    private:
      size_t _size = 0;

    public:
      std::vector<vec3f> lower;
      std::vector<float> width;
      std::vector<unsigned char> value;
      std::vector<range1f> range;

      HtgVoxels() = default;

      HtgVoxels(size_t size, size_t stride)
          : _size(size),
            lower(size),
            width(size),
            value(size * stride),
            range(size)
      {
      }

      template <typename vtype>
      void setVoxel(size_t index, const vec3f &l, float w, vtype v, range1f r)
      {
        *(lower.data() + index) = l;
        *(width.data() + index) = w;
        vtype *valueTyped = (vtype *)(value.data() + index * sizeof(vtype));
        *valueTyped = v;
        *(range.data() + index) = r;
      }

      template <typename vtype>
      void addVoxel(const vec3f &l, float w, vtype v, range1f r)
      {
        const auto index = _size;
        ++_size;
        lower.push_back(l);
        width.push_back(w);
        range.push_back(r);
        for (int i = 0; i < sizeof(vtype); ++i)
        {
          value.push_back(0);
        }
        vtype *valueTyped = (vtype *)(value.data() + index * sizeof(vtype));
        *valueTyped = v;
      }

      size_t size() const
      {
        return _size;
      }

      void filter()
      {
        HtgVoxels tmp;
        const size_t stride = value.size() / this->_size;
        // TODO do a SIMD version reduction
        for (size_t a = 0; a != this->_size; ++a)
        {
          if (this->width[a] > 0.f)
          {
            tmp.lower.push_back(this->lower[a]);
            tmp.width.push_back(this->width[a]);
            tmp.range.push_back(this->range[a]);
            for (size_t i = 0; i < stride; ++i)
              tmp.value.push_back(this->value[a * stride + i]);
            ++tmp._size;
          }
        }
        this->_size = tmp._size;
        this->lower.swap(tmp.lower);
        this->width.swap(tmp.width);
        this->value.swap(tmp.value);
        this->range.swap(tmp.range);
      }
    };

    template <typename vtype>
    struct HtgBuilder
    {
      std::vector<HtgNode> data;
      box3f actualBounds; // grid world bound
      box3f extendBounds; // extend the dimension to pow of 2
      size_t numVoxels;

      HtgBuilder(const box3f &bounds, const HtgVoxels &voxels, size_t numVoxels)
          : allVoxels(voxels), numVoxels(numVoxels)
      {
        actualBounds = bounds;
        extendBounds.lower = actualBounds.lower;
        extendBounds.upper =
            reduce_max(vec3f(roundToPow2(actualBounds.upper.x),
                             roundToPow2(actualBounds.upper.y),
                             roundToPow2(actualBounds.upper.z)));
      }

      void build()
      {
        data.push_back(HtgNode(1));
        recursive(0, extendBounds, numVoxels);
        experiment();
      }

      void print();

      void experiment();

    private:
      const HtgVoxels &allVoxels;
      // range1f overallValueRange = range1f(rkcommon::math::empty);
      size_t actualNumOfNodes = 0;
      size_t extendNumOfNodes = 0;
      size_t deactivatedNodes = 0;
      size_t recursive(const size_t nodeId,
                       const box3f &voxelBounds,
                       const size_t voxelNum,
                       const size_t *voxelIdx = NULL);
      double valueToDouble(size_t index) const
      {
        return *(vtype *)(this->allVoxels.value.data() + index * sizeof(vtype));
      }

      void deactivate_this_node(size_t i, int level, const int max_level)
      {
        if (isInvalid(data[i]))
        {
          return;
        }
        if (!isTerminatingNode(data[i]))
        {
          const auto offset = getChildOffset(data[i]);
          for (int k = 0; k < 8; ++k)
          {
            deactivate_this_node(i + offset + k, level + 1, max_level);
          }
          if (level >= max_level)
          {
            ++deactivatedNodes;
            const range1f r = getValueRange(data[i]);
            setAsEvicted(data[i], true);
            setValue(data[i], r.center());
          }
        }
      }
    };

    template <typename vtype>
    inline void HtgBuilder<vtype>::print()
    {
      printf("-- node number: %zd\n", data.size());
      printf("-- number of bricks: %zd\n", (data.size() - 1) / 8);
      printf("-- data usage: %f\n", actualNumOfNodes / (float)extendNumOfNodes);
      printf("-- deactivated nodes: %zd\n\n", deactivatedNodes);
#if 0
      for (size_t i = 0; i < data.size(); i++) {
        printf("%zd ", i);
        if (isInvalid(data[i])) {
          printf("Invalid: --------------\n");
        } else if (isValueRangeNode(data[i])) {
          printf("Value Range: vRange: [%f, %f]\n",
                 getValueRange(data[i]).lower,
                 getValueRange(data[i]).upper);
        } else if (isTerminatingNode(data[i])) {
          printf("Terminating Node: value: %f\n", getValue(data[i]));
        } else {
          printf("Inner Node: children offset: %zd, children mask: %u, ",
                 getChildOffset(data[i]),
                 getChildMask(data[i]));
          const range1f valueRange = getValueRange(data[i]);
          printf("range: [%f, %f]\n", valueRange.lower, valueRange.upper);
        }
        if (i % 9 == 0)
          printf("\n");
      }
#endif
    }

    template <typename vtype>
    inline size_t HtgBuilder<vtype>::recursive(
        const size_t nodeId, /* subtree root */
        const box3f &voxelBounds,
        const size_t voxelNum,
        const size_t *voxelIdx)
    {
      assert(voxelNum > 1);

      const vec3f center = voxelBounds.center();

      // compute the boxes for all 8 children
      box3f subBounds[8];
      for (int i = 0; i < 8; i++)
      {
        subBounds[i].lower = vec3f((i & 1) ? center.x : voxelBounds.lower.x,
                                   (i & 2) ? center.y : voxelBounds.lower.y,
                                   (i & 4) ? center.z : voxelBounds.lower.z);
        subBounds[i].upper = subBounds[i].lower + 0.5 * voxelBounds.size();
      }

      // TODO optimize this section
      std::vector<size_t> subVoxelIndex[8]; // voxels within each child the
      range1f subVoxelValueRange;
      for (size_t i = 0; i < voxelNum; i++)
      {
        const size_t cVoxelIdx = (nodeId == 0) ? i : voxelIdx[i];
        const vec3f voxelCenter = this->allVoxels.lower[cVoxelIdx] -
                                  this->actualBounds.lower +
                                  0.5 * this->allVoxels.width[cVoxelIdx];
        uint8_t childIdx = 0;
        childIdx |= voxelCenter.x < center.x ? 0 : 1;
        childIdx |= voxelCenter.y < center.y ? 0 : 2;
        childIdx |= voxelCenter.z < center.z ? 0 : 4;
        subVoxelIndex[childIdx].push_back(cVoxelIdx);
        subVoxelValueRange.extend(this->allVoxels.range[cVoxelIdx]);
        // overallValueRange.extend(this->allVoxels.range[cVoxelIdx]);
      }

      // the child offset of the subtree root
      const size_t childOffset = data.size() - nodeId;

      int childCount = 0;
      uint32_t childMask = 0;
      for (int i = 0; i < 8; i++)
      {
        if (subVoxelIndex[i].size() != 0)
        {
          childMask |= 256 >> (8 - i);
          ++childCount;
        }
      }

      // push children node into the buffer, initialize later.
      this->actualNumOfNodes += childCount;
      this->extendNumOfNodes += 9; /* 8 children + value range */
      for (int i = 0; i < 8; i++)
      {
        data.push_back(HtgNode());
        data.back().set(-1); // by default, nodes are invalid
      }
      data.push_back(HtgNode()); // value range
      data.back().set(0);        // by default, nodes are invalid
      setAsValueRangeNode(data.back(), true);
      setValueRange(data.back(), subVoxelValueRange);

      // initialize children of the current node
      for (int i = 0; i < 8; i++)
      {
        const int idx = i;
        const size_t childIndex = nodeId + childOffset + i;
        if (subVoxelIndex[idx].size() > 0)
        {
          data[childIndex].set(0); // validate the node
          if (subVoxelIndex[idx].size() == 1)
          {
            const double value = valueToDouble(subVoxelIndex[idx][0]);
            setAsLeaf(data[childIndex], 1);
            setValue(data[childIndex], value);
          }
          else
          {
            const size_t offset = recursive(nodeId + childOffset + i,
                                            subBounds[idx],
                                            subVoxelIndex[idx].size(),
                                            subVoxelIndex[idx].data());
            setChildOffset(data[childIndex], offset);
          }
        }
      }

      setChildMask(data[nodeId], childMask);

      return childOffset;
    }

    template <typename vtype>
    inline void HtgBuilder<vtype>::experiment()
    {
      // deactivate the subtree starting from this index
      // deactivate_this_node(0, 0, 3);
    }

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
      size_t p = 0;
      size_t map_size = 0;
      size_t file_size = 0;
    };

    inline FileMap filemap_write_create(const std::string &filename, size_t requested_size)
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
        throw std::runtime_error("Error opening file for writing");
      }

      // Stretch the file size to the size of the (mmapped) array of char

      if (lseek(fd, requested_size - 1, SEEK_SET) == -1)
      {
        close(fd);
        throw std::runtime_error("Error calling lseek() to 'stretch' the file");
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
        throw std::runtime_error("Error writing last byte of the file");
      }

      // Now the file is ready to be mmapped.
      map = (char *)mmap(0, requested_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
      if (map == MAP_FAILED)
      {
        close(fd);
        throw std::runtime_error("Error mmapping the file");
      }

      ret.map_size = requested_size;
      ret.file_size = requested_size;
      return ret;
    }

    inline FileMap filemap_read_create(const std::string &filename, size_t requested_size = 0)
    {
      FileMap ret;
      ret.type = FileMap::BINARY_READ;

      int &fd = ret.fd;
      char *&map = ret.map;

      fd = open(filename.c_str(), O_RDONLY, (mode_t)0600);

      if (fd == -1)
      {
        perror("Error opening file for writing");
        exit(EXIT_FAILURE);
      }

      struct stat fileInfo = {0};

      if (fstat(fd, &fileInfo) == -1)
      {
        perror("Error getting the file size");
        exit(EXIT_FAILURE);
      }

      if (fileInfo.st_size == 0)
      {
        fprintf(stderr, "Error: File is empty, nothing to do\n");
        exit(EXIT_FAILURE);
      }

      printf("File size is %ji\n", (intmax_t)fileInfo.st_size);

      map = (char *)mmap(0, fileInfo.st_size, PROT_READ, MAP_SHARED, fd, 0);
      if (map == MAP_FAILED)
      {
        close(fd);
        perror("Error mmapping the file");
        exit(EXIT_FAILURE);
      }

      assert(requested_size <= fileInfo.st_size);
      ret.map_size = requested_size == 0 ? fileInfo.st_size : requested_size;
      ret.file_size = fileInfo.st_size;
      return ret;
    }

    inline void filemap_write(FileMap &file, const void *data, const size_t bytes)
    {
      assert(file.type == FileMap::BINARY_WRITE);
      assert(bytes <= file.map_size);

      printf("Writing %zu bytes\n", bytes);

      // Write data to in-core memory
      const char *text = (const char *)data;
      for (size_t i = 0; i < bytes; i++)
      {
        file.map[file.p + i] = text[i];
      }

      file.p += bytes;
    }

    inline void filemap_read(FileMap &file, void *data, const size_t bytes)
    {
      assert(file.type == FileMap::BINARY_READ);
      assert(bytes <= file.map_size);

      printf("Read %zu bytes\n", bytes);

      char *text = (char *)data;
      for (size_t i = 0; i < bytes; i++)
      {
        text[i] = file.map[file.p + i];
      }

      file.p += bytes;
    }

    inline void filemap_close(FileMap &file)
    {
      // Flush data now to disk
      if (file.type == FileMap::BINARY_WRITE)
      {
        if (msync(file.map, file.map_size, MS_SYNC) == -1)
        {
          throw std::runtime_error("Could not sync the file to disk");
        }
      }

      // Don't forget to free the mmapped memory
      if (munmap(file.map, file.map_size) == -1)
      {
        close(file.fd);
        throw std::runtime_error("Error un-mmapping the file");
      }

      // Un-mmaping doesn't close the file, so we still need to do that.
      close(file.fd);
    }

  } // namespace ispc_driver
} // namespace openvkl
#endif // defined(__cplusplus)
