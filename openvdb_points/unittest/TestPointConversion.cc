///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015-2016 Double Negative Visual Effects
//
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
//
// Redistributions of source code must retain the above copyright
// and license notice and the following restrictions and disclaimer.
//
// *     Neither the name of Double Negative Visual Effects nor the names
// of its contributors may be used to endorse or promote products derived
// from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// IN NO EVENT SHALL THE COPYRIGHT HOLDERS' AND CONTRIBUTORS' AGGREGATE
// LIABILITY FOR ALL CLAIMS REGARDLESS OF THEIR BASIS EXCEED US$250.00.
//
///////////////////////////////////////////////////////////////////////////


#include <cppunit/extensions/HelperMacros.h>

#include <openvdb_points/tools/PointDataGrid.h>
#include <openvdb_points/tools/PointAttribute.h>
#include <openvdb_points/tools/PointConversion.h>
#include <openvdb_points/tools/PointCount.h>
#include <openvdb_points/tools/PointGroup.h>
#include <openvdb_points/openvdb.h>

using namespace openvdb;
using namespace openvdb::tools;

class TestPointConversion: public CppUnit::TestCase
{
public:
    virtual void setUp() { openvdb::initialize(); openvdb::points::initialize(); }
    virtual void tearDown() { openvdb::uninitialize(); openvdb::points::uninitialize(); }

    CPPUNIT_TEST_SUITE(TestPointConversion);
    CPPUNIT_TEST(testPointConversion);
    CPPUNIT_TEST(testStride);
    CPPUNIT_TEST(testAutoVoxelSize);

    CPPUNIT_TEST_SUITE_END();

    void testPointConversion();
    void testStride();
    void testAutoVoxelSize();

}; // class TestPointConversion

CPPUNIT_TEST_SUITE_REGISTRATION(TestPointConversion);


// Simple Attribute Wrapper
template <typename T>
struct AttributeWrapper
{
    typedef T ValueType;
    typedef T PosType;
    typedef T value_type;

    struct Handle
    {
        Handle(AttributeWrapper<T>& attribute)
            : mBuffer(attribute.mAttribute)
            , mStride(attribute.mStride) { }

        template <typename ValueType>
        void set(size_t n, openvdb::Index m, const ValueType& value) {
            mBuffer[n * mStride + m] = value;
        }

        template <typename ValueType>
        void set(size_t n, openvdb::Index m, const openvdb::math::Vec3<ValueType>& value) {
            mBuffer[n * mStride + m] = value;
        }

    private:
        std::vector<T>& mBuffer;
        Index mStride;
    }; // struct Handle

    explicit AttributeWrapper(const Index stride) : mStride(stride) { }

    void expand() { }
    void compact() { }

    void resize(const size_t n) { mAttribute.resize(n); }
    size_t size() const { return mAttribute.size(); }

    std::vector<T>& buffer() { return mAttribute; }

    template <typename ValueT>
    void get(ValueT& value, size_t n, openvdb::Index m = 0) const { value = mAttribute[n * mStride + m]; }
    template <typename ValueT>
    void getPos(size_t n, ValueT& value) const { this->get<ValueT>(value, n); }

private:
    std::vector<T> mAttribute;
    Index mStride;
}; // struct AttributeWrapper


struct GroupWrapper
{
    GroupWrapper() { }

    void setOffsetOn(openvdb::Index index) {
        mGroup[index] = short(1);
    }

    void finalize() { }

    void resize(const size_t n) { mGroup.resize(n, short(0)); }
    size_t size() const { return mGroup.size(); }

    std::vector<short>& buffer() { return mGroup; }

private:
    std::vector<short> mGroup;
}; // struct GroupWrapper


struct PointData
{
    int id;
    Vec3f position;
    Vec3i xyz;
    float uniform;
    openvdb::Name string;
    short group;

    bool operator<(const PointData& other) const { return id < other.id; }
}; // PointData


// Generate random points by uniformly distributing points
// on a unit-sphere.
void genPoints( const int numPoints, const double scale, const bool stride,
                AttributeWrapper<Vec3f>& position,
                AttributeWrapper<int>& xyz,
                AttributeWrapper<int>& id,
                AttributeWrapper<float>& uniform,
                AttributeWrapper<openvdb::Name>& string,
                GroupWrapper& group)
{
    // init
    openvdb::math::Random01 randNumber(0);
    const int n = int(std::sqrt(double(numPoints)));
    const double xScale = (2.0 * M_PI) / double(n);
    const double yScale = M_PI / double(n);

    double x, y, theta, phi;
    openvdb::Vec3f pos;

    position.resize(n*n);
    xyz.resize(stride ? n*n*3 : 1);
    id.resize(n*n);
    uniform.resize(n*n);
    string.resize(n*n);
    group.resize(n*n);

    AttributeWrapper<Vec3f>::Handle positionHandle(position);
    AttributeWrapper<int>::Handle xyzHandle(xyz);
    AttributeWrapper<int>::Handle idHandle(id);
    AttributeWrapper<float>::Handle uniformHandle(uniform);
    AttributeWrapper<openvdb::Name>::Handle stringHandle(string);

    int i = 0;

    // loop over a [0 to n) x [0 to n) grid.
    for (int a = 0; a < n; ++a) {
        for (int b = 0; b < n; ++b) {

            // jitter, move to random pos. inside the current cell
            x = double(a) + randNumber();
            y = double(b) + randNumber();

            // remap to a lat/long map
            theta = y * yScale; // [0 to PI]
            phi   = x * xScale; // [0 to 2PI]

            // convert to cartesian coordinates on a unit sphere.
            // spherical coordinate triplet (r=1, theta, phi)
            pos[0] = std::sin(theta)*std::cos(phi)*scale;
            pos[1] = std::sin(theta)*std::sin(phi)*scale;
            pos[2] = std::cos(theta)*scale;

            positionHandle.set(i, /*stride*/0, pos);
            idHandle.set(i, /*stride*/0, i);
            uniformHandle.set(i, /*stride*/0, 100.0f);

            if (stride)
            {
                xyzHandle.set(i, 0, i);
                xyzHandle.set(i, 1, i*i);
                xyzHandle.set(i, 2, i*i*i);
            }

            // add points with even id to the group
            if ((i % 2) == 0) {
                group.setOffsetOn(i);
                stringHandle.set(i, /*stride*/0, "testA");
            }
            else {
                stringHandle.set(i, /*stride*/0, "testB");
            }

            i++;
        }
    }
}


////////////////////////////////////////


void
TestPointConversion::testPointConversion()
{
    // Define and register some common attribute types
    typedef TypedAttributeArray<int32_t>        AttributeI;
    typedef TypedAttributeArray<float>          AttributeF;
    typedef TypedAttributeArray<openvdb::Vec3s> AttributeVec3s;

    AttributeI::registerType();
    AttributeF::registerType();
    AttributeVec3s::registerType();

    // generate points

    const unsigned long count(40000);

    AttributeWrapper<Vec3f> position(1);
    AttributeWrapper<int> xyz(1);
    AttributeWrapper<int> id(1);
    AttributeWrapper<float> uniform(1);
    AttributeWrapper<openvdb::Name> string(1);
    GroupWrapper group;

    genPoints(count, /*scale=*/ 100.0, /*stride=*/false,
                position, xyz, id, uniform, string, group);

    CPPUNIT_ASSERT_EQUAL(position.size(), count);
    CPPUNIT_ASSERT_EQUAL(id.size(), count);
    CPPUNIT_ASSERT_EQUAL(uniform.size(), count);
    CPPUNIT_ASSERT_EQUAL(string.size(), count);
    CPPUNIT_ASSERT_EQUAL(group.size(), count);

    // convert point positions into a Point Data Grid

    const float voxelSize = 1.0f;
    openvdb::math::Transform::Ptr transform(openvdb::math::Transform::createLinearTransform(voxelSize));

    PointIndexGrid::Ptr pointIndexGrid = createPointIndexGrid<PointIndexGrid>(position, *transform);
    PointDataGrid::Ptr pointDataGrid = createPointDataGrid<NullCodec, PointDataGrid>(*pointIndexGrid, position, *transform);

    PointIndexTree& indexTree = pointIndexGrid->tree();
    PointDataTree& tree = pointDataGrid->tree();

    // add id and populate

    appendAttribute<AttributeI>(tree, "id");
    populateAttribute<PointDataTree, PointIndexTree, AttributeWrapper<int>, false>(tree, indexTree, "id", id);

    // add uniform and populate

    appendAttribute<AttributeF>(tree, "uniform");
    populateAttribute<PointDataTree, PointIndexTree, AttributeWrapper<float>, false>(tree, indexTree, "uniform", uniform);

    // add string and populate

    appendAttribute<StringAttributeArray>(tree, "string");

    // extract the metadata and reset the descriptors
    PointDataTree::LeafIter leafIter = tree.beginLeaf();
    const AttributeSet::Descriptor& descriptor = leafIter->attributeSet().descriptor();
    AttributeSet::Descriptor::Ptr newDescriptor(new AttributeSet::Descriptor(descriptor));
    MetaMap& metadata = newDescriptor->getMetadata();
    for (; leafIter; ++leafIter) {
        leafIter->resetDescriptor(newDescriptor);
    }

    // insert the required strings into the metadata
    StringMetaInserter inserter(metadata);
    inserter.insert("testA");
    inserter.insert("testB");

    populateAttribute<PointDataTree, PointIndexTree, AttributeWrapper<openvdb::Name>, false>(tree, indexTree, "string", string);

    // add group and set membership

    appendGroup(tree, "test");
    setGroup(tree, indexTree, group.buffer(), "test");

    CPPUNIT_ASSERT_EQUAL(indexTree.leafCount(), tree.leafCount());

    // create accessor and iterator for Point Data Tree

    PointDataTree::LeafCIter leafCIter = tree.cbeginLeaf();

    CPPUNIT_ASSERT_EQUAL((unsigned long) 5, leafCIter->attributeSet().size());

    CPPUNIT_ASSERT(leafCIter->attributeSet().find("id") != AttributeSet::INVALID_POS);
    CPPUNIT_ASSERT(leafCIter->attributeSet().find("uniform") != AttributeSet::INVALID_POS);
    CPPUNIT_ASSERT(leafCIter->attributeSet().find("P") != AttributeSet::INVALID_POS);
    CPPUNIT_ASSERT(leafCIter->attributeSet().find("string") != AttributeSet::INVALID_POS);

    const size_t idIndex = leafCIter->attributeSet().find("id");
    const size_t uniformIndex = leafCIter->attributeSet().find("uniform");
    const size_t stringIndex = leafCIter->attributeSet().find("string");
    const AttributeSet::Descriptor::GroupIndex groupIndex = leafCIter->attributeSet().groupIndex("test");

    // convert back into linear point attribute data

    AttributeWrapper<Vec3f> outputPosition(1);
    AttributeWrapper<int> outputId(1);
    AttributeWrapper<float> outputUniform(1);
    AttributeWrapper<openvdb::Name> outputString(1);
    GroupWrapper outputGroup;

    // test offset the whole point block by an arbitrary amount

    Index64 startOffset = 10;

    outputPosition.resize(startOffset + position.size());
    outputId.resize(startOffset + id.size());
    outputUniform.resize(startOffset + uniform.size());
    outputString.resize(startOffset + string.size());
    outputGroup.resize(startOffset + group.size());

    std::vector<Index64> pointOffsets;
    getPointOffsets(pointOffsets, tree);

    convertPointDataGridPosition(outputPosition, *pointDataGrid, pointOffsets, startOffset);
    convertPointDataGridAttribute(outputId, tree, pointOffsets, startOffset, idIndex);
    convertPointDataGridAttribute(outputUniform, tree, pointOffsets, startOffset, uniformIndex);
    convertPointDataGridAttribute(outputString, tree, pointOffsets, startOffset, stringIndex);
    convertPointDataGridGroup(outputGroup, tree, pointOffsets, startOffset, groupIndex);

    // pack and sort the new buffers based on id

    std::vector<PointData> pointData;

    pointData.resize(count);

    for (unsigned int i = 0; i < count; i++) {
        pointData[i].id = outputId.buffer()[startOffset + i];
        pointData[i].position = outputPosition.buffer()[startOffset + i];
        pointData[i].uniform = outputUniform.buffer()[startOffset + i];
        pointData[i].string = outputString.buffer()[startOffset + i];
        pointData[i].group = outputGroup.buffer()[startOffset + i];
    }

    std::sort(pointData.begin(), pointData.end());

    // compare old and new buffers

    for (unsigned int i = 0; i < count; i++)
    {
        CPPUNIT_ASSERT_EQUAL(id.buffer()[i], pointData[i].id);
        CPPUNIT_ASSERT_EQUAL(group.buffer()[i], pointData[i].group);
        CPPUNIT_ASSERT_EQUAL(uniform.buffer()[i], pointData[i].uniform);
        CPPUNIT_ASSERT_EQUAL(string.buffer()[i], pointData[i].string);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(position.buffer()[i].x(), pointData[i].position.x(), /*tolerance=*/1e-6);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(position.buffer()[i].y(), pointData[i].position.y(), /*tolerance=*/1e-6);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(position.buffer()[i].z(), pointData[i].position.z(), /*tolerance=*/1e-6);
    }

    // convert based on even group

    const unsigned long halfCount = count / 2;

    outputPosition.resize(startOffset + halfCount);
    outputId.resize(startOffset + halfCount);
    outputUniform.resize(startOffset + halfCount);
    outputString.resize(startOffset + halfCount);
    outputGroup.resize(startOffset + halfCount);

    std::vector<Name> includeGroups;
    includeGroups.push_back("test");

    pointOffsets.clear();
    getPointOffsets(pointOffsets, tree, includeGroups);

    convertPointDataGridPosition(outputPosition, *pointDataGrid, pointOffsets, startOffset, includeGroups);
    convertPointDataGridAttribute(outputId, tree, pointOffsets, startOffset, idIndex, /*stride*/1, includeGroups);
    convertPointDataGridAttribute(outputUniform, tree, pointOffsets, startOffset, uniformIndex, /*stride*/1, includeGroups);
    convertPointDataGridAttribute(outputString, tree, pointOffsets, startOffset, stringIndex, /*stride*/1, includeGroups);
    convertPointDataGridGroup(outputGroup, tree, pointOffsets, startOffset, groupIndex, includeGroups);

    CPPUNIT_ASSERT_EQUAL(size_t(outputPosition.size() - startOffset), size_t(halfCount));
    CPPUNIT_ASSERT_EQUAL(size_t(outputId.size() - startOffset), size_t(halfCount));
    CPPUNIT_ASSERT_EQUAL(size_t(outputUniform.size() - startOffset), size_t(halfCount));
    CPPUNIT_ASSERT_EQUAL(size_t(outputString.size() - startOffset), size_t(halfCount));
    CPPUNIT_ASSERT_EQUAL(size_t(outputGroup.size() - startOffset), size_t(halfCount));

    pointData.clear();

    for (unsigned int i = 0; i < halfCount; i++) {
        PointData data;
        data.id = outputId.buffer()[startOffset + i];
        data.position = outputPosition.buffer()[startOffset + i];
        data.uniform = outputUniform.buffer()[startOffset + i];
        data.string = outputString.buffer()[startOffset + i];
        data.group = outputGroup.buffer()[startOffset + i];
        pointData.push_back(data);
    }

    std::sort(pointData.begin(), pointData.end());

    // compare old and new buffers

    for (unsigned int i = 0; i < halfCount; i++)
    {
        CPPUNIT_ASSERT_EQUAL(id.buffer()[i*2], pointData[i].id);
        CPPUNIT_ASSERT_EQUAL(group.buffer()[i*2], pointData[i].group);
        CPPUNIT_ASSERT_EQUAL(uniform.buffer()[i*2], pointData[i].uniform);
        CPPUNIT_ASSERT_EQUAL(string.buffer()[i*2], pointData[i].string);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(position.buffer()[i*2].x(), pointData[i].position.x(), /*tolerance=*/1e-6);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(position.buffer()[i*2].y(), pointData[i].position.y(), /*tolerance=*/1e-6);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(position.buffer()[i*2].z(), pointData[i].position.z(), /*tolerance=*/1e-6);
    }
}


////////////////////////////////////////


void
TestPointConversion::testStride()
{
    // Define and register some common attribute types
    typedef TypedAttributeArray<int32_t>        AttributeI;
    typedef TypedAttributeArray<float>          AttributeF;
    typedef TypedAttributeArray<openvdb::Vec3s> AttributeVec3s;

    AttributeI::registerType();
    AttributeF::registerType();
    AttributeVec3s::registerType();

    // generate points

    const unsigned long count(40000);

    AttributeWrapper<Vec3f> position(1);
    AttributeWrapper<int> xyz(3);
    AttributeWrapper<int> id(1);
    AttributeWrapper<float> uniform(1);
    AttributeWrapper<openvdb::Name> string(1);
    GroupWrapper group;

    genPoints(count, /*scale=*/ 100.0, /*stride=*/true,
                position, xyz, id, uniform, string, group);

    CPPUNIT_ASSERT_EQUAL(position.size(), count);
    CPPUNIT_ASSERT_EQUAL(xyz.size(), count*3);
    CPPUNIT_ASSERT_EQUAL(id.size(), count);

    // convert point positions into a Point Data Grid

    const float voxelSize = 1.0f;
    openvdb::math::Transform::Ptr transform(openvdb::math::Transform::createLinearTransform(voxelSize));

    PointIndexGrid::Ptr pointIndexGrid = createPointIndexGrid<PointIndexGrid>(position, *transform);
    PointDataGrid::Ptr pointDataGrid = createPointDataGrid<NullCodec, PointDataGrid>(*pointIndexGrid, position, *transform);

    PointIndexTree& indexTree = pointIndexGrid->tree();
    PointDataTree& tree = pointDataGrid->tree();

    // add id and populate

    appendAttribute<AttributeI>(tree, "id");
    populateAttribute<PointDataTree, PointIndexTree, AttributeWrapper<int>, false>(tree, indexTree, "id", id);

    // add xyz and populate

    appendAttribute<AttributeI>(tree, "xyz", /*stride=*/3);
    populateAttribute<PointDataTree, PointIndexTree, AttributeWrapper<int>, true>(tree, indexTree, "xyz", xyz, /*stride=*/3);

    // create accessor and iterator for Point Data Tree

    PointDataTree::LeafCIter leafCIter = tree.cbeginLeaf();

    CPPUNIT_ASSERT_EQUAL((unsigned long) 3, leafCIter->attributeSet().size());

    CPPUNIT_ASSERT(leafCIter->attributeSet().find("id") != AttributeSet::INVALID_POS);
    CPPUNIT_ASSERT(leafCIter->attributeSet().find("P") != AttributeSet::INVALID_POS);
    CPPUNIT_ASSERT(leafCIter->attributeSet().find("xyz") != AttributeSet::INVALID_POS);

    const size_t idIndex = leafCIter->attributeSet().find("id");
    const size_t xyzIndex = leafCIter->attributeSet().find("xyz");

    // convert back into linear point attribute data

    AttributeWrapper<Vec3f> outputPosition(1);
    AttributeWrapper<int> outputXyz(3);
    AttributeWrapper<int> outputId(1);

    // test offset the whole point block by an arbitrary amount

    Index64 startOffset = 10;

    outputPosition.resize(startOffset + position.size());
    outputXyz.resize((startOffset + id.size())*3);
    outputId.resize(startOffset + id.size());

    std::vector<Index64> pointOffsets;
    getPointOffsets(pointOffsets, tree);

    convertPointDataGridPosition(outputPosition, *pointDataGrid, pointOffsets, startOffset);
    convertPointDataGridAttribute(outputId, tree, pointOffsets, startOffset, idIndex);
    convertPointDataGridAttribute(outputXyz, tree, pointOffsets, startOffset, xyzIndex, /*stride=*/3);

    // pack and sort the new buffers based on id

    std::vector<PointData> pointData;

    pointData.resize(count);

    for (unsigned int i = 0; i < count; i++) {
        pointData[i].id = outputId.buffer()[startOffset + i];
        pointData[i].position = outputPosition.buffer()[startOffset + i];
        for (unsigned int j = 0; j < 3; j++) {
            pointData[i].xyz[j] = outputXyz.buffer()[startOffset * 3 + i * 3 + j];
        }
    }

    std::sort(pointData.begin(), pointData.end());

    // compare old and new buffers

    for (unsigned int i = 0; i < count; i++)
    {
        CPPUNIT_ASSERT_EQUAL(id.buffer()[i], pointData[i].id);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(position.buffer()[i].x(), pointData[i].position.x(), /*tolerance=*/1e-6);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(position.buffer()[i].y(), pointData[i].position.y(), /*tolerance=*/1e-6);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(position.buffer()[i].z(), pointData[i].position.z(), /*tolerance=*/1e-6);
        CPPUNIT_ASSERT_EQUAL(Vec3i(xyz.buffer()[i*3], xyz.buffer()[i*3+1], xyz.buffer()[i*3+2]), pointData[i].xyz);
    }
}


////////////////////////////////////////


void
TestPointConversion::testAutoVoxelSize()
{
    struct Local {

        static PointDataGrid::Ptr genPointsGrid(const float voxelSize, const AttributeWrapper<Vec3f>& positions)
        {
            math::Transform::Ptr transform(math::Transform::createLinearTransform(voxelSize));
            PointIndexGrid::Ptr pointIndexGrid = createPointIndexGrid<PointIndexGrid>(positions, *transform);
            return createPointDataGrid<NullCodec, PointDataGrid>(*pointIndexGrid, positions, *transform);
        }
    };

    AttributeWrapper<Vec3f> position(/*stride*/1);

    static const Vec3R min(-std::numeric_limits<Real>::max());
    static const Vec3R max(std::numeric_limits<Real>::max());

    // test with no positions

    {
        const float voxelSize = autoVoxelSize(position, /*points per voxel*/8);
        CPPUNIT_ASSERT_EQUAL(voxelSize, 0.1f);
    }

    // test with one point

    {
        position.resize(1);
        AttributeWrapper<Vec3f>::Handle positionHandle(position);
        positionHandle.set(0, 0, Vec3f(0.0f));

        const float voxelSize = autoVoxelSize(position, /*points per voxel*/8);
        CPPUNIT_ASSERT_EQUAL(voxelSize, 0.1f);
    }

    // test with n points, where n > 1 && n <= num points per voxel

    {
        position.resize(7);
        AttributeWrapper<Vec3f>::Handle positionHandle(position);
        positionHandle.set(0, 0, Vec3f(-8.6f, 0.0f,-23.8f));
        positionHandle.set(1, 0, Vec3f( 8.6f, 7.8f, 23.8f));

        for(size_t i = 2; i < 7; ++ i)
            positionHandle.set(i, 0, Vec3f(0.0f));

        float voxelSize = autoVoxelSize(position, /*points per voxel*/8);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(voxelSize, 18.5528f, /*tolerance=*/1e-4);

        voxelSize = autoVoxelSize(position, /*points per voxel*/1);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(voxelSize, 5.51306f, /*tolerance=*/1e-4);

        // test decimal place accuracy

        voxelSize = autoVoxelSize(position, /*points per voxel*/1, max, min, math::Mat4d::identity(), 10);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(voxelSize, 5.5130610466f, /*tolerance=*/1e-9);

        voxelSize = autoVoxelSize(position, /*points per voxel*/1, max, min, math::Mat4d::identity(), 1);
        CPPUNIT_ASSERT_EQUAL(voxelSize, 5.5f);

        voxelSize = autoVoxelSize(position, /*points per voxel*/1, max, min, math::Mat4d::identity(), 0);
        CPPUNIT_ASSERT_EQUAL(voxelSize, 6.0f);
    }

    // test where all points line on a plane

    {
        position.resize(5);
        AttributeWrapper<Vec3f>::Handle positionHandle(position);
        positionHandle.set(0, 0, Vec3f(0.0f, 0.0f, 10.0f));
        positionHandle.set(1, 0, Vec3f(0.0f, 0.0f, -10.0f));
        positionHandle.set(2, 0, Vec3f(20.0f, 0.0f, -10.0f));
        positionHandle.set(3, 0, Vec3f(20.0f, 0.0f, 10.0f));
        positionHandle.set(4, 0, Vec3f(10.0f, 0.0f, 0.0f));

        float voxelSize = autoVoxelSize(position, /*points per voxel*/5);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(voxelSize, 3.41995f, /*tolerance=*/1e-4);

        voxelSize = autoVoxelSize(position, /*points per voxel*/1);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(voxelSize, 2.0f, /*tolerance=*/1e-4);
    }

    // limits test

    {
        const openvdb::Vec3f smallest(std::numeric_limits<float>::min());

        position.resize(2);
        AttributeWrapper<Vec3f>::Handle positionHandle(position);
        positionHandle.set(0, 0, Vec3f(0.0f));
        positionHandle.set(1, 0, Vec3f(smallest));

        float voxelSize = autoVoxelSize(position, /*points per voxel*/2);
        CPPUNIT_ASSERT_EQUAL(voxelSize, 0.1f);

        // algorithm won't be able to calculate a voxel size for 1 point per voxel
        // but should still return a value result that isn't the default

        voxelSize = autoVoxelSize(position, /*points per voxel*/1);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(voxelSize, 0.063f, /*tolerance=*/1e-4);

        PointDataGrid::Ptr grid = Local::genPointsGrid(voxelSize, position);
        CPPUNIT_ASSERT_EQUAL(grid->activeVoxelCount(), Index64(1));
    }

    {
        const openvdb::Vec3f smallest(std::numeric_limits<float>::min());

        position.resize(100000);
        AttributeWrapper<Vec3f>::Handle positionHandle(position);
        positionHandle.set(0, 0, Vec3f(0.0f));

        for(size_t i = 1; i < 100000; ++ i)
            positionHandle.set(i, 0, Vec3f(smallest * i));

        float voxelSize = autoVoxelSize(position, /*points per voxel*/10);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(voxelSize, 0.00022f, /*tolerance=*/1e-4);

        // algorithm won't be able to calculate a voxel size for 1 point per voxel
        // but should still return a value result that isn't the default

        voxelSize = autoVoxelSize(position, /*points per voxel*/1);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(voxelSize, 5e-5, /*tolerance=*/1e-6);

        PointDataGrid::Ptr grid = Local::genPointsGrid(voxelSize, position);
        CPPUNIT_ASSERT_EQUAL(grid->activeVoxelCount(), Index64(1));

        // check zero decimal place still returns valid result

        voxelSize = autoVoxelSize(position, /*points per voxel*/1, max, min, math::Mat4d::identity(), 0);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(voxelSize, 5e-5, /*tolerance=*/1e-6);
    }

    // random position generation within two bounds of equal size.
    // This test distributes 1000 points within a 1x1x1 box centered at (0,0,0)
    // and another 1000 points within a separate 1x1x1 box centered at (20,20,20).
    // Points are randomly positioned however can be defined as having a stochastic
    // distribution. Tests that sparsity between these data sets causes no issues
    // and that autoVoxelSize produces accurate results

    {
        position.resize(2000);
        AttributeWrapper<Vec3f>::Handle positionHandle(position);
        openvdb::math::Random01 randNumber(0);

        // positions between -0.5 and 0.5

        for(size_t i = 0; i < 1000; ++ i) {
            const Vec3f pos(randNumber() - 0.5f);
            positionHandle.set(i, 0, pos);
        }

        // positions between 19.5 and 20.5

        for(size_t i = 1000; i < 2000; ++ i) {
            const Vec3f pos(randNumber() - 0.5f + 20.0f);
            positionHandle.set(i, 0, pos);
        }

        float voxelSize = autoVoxelSize(position, /*points per voxel*/1);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(voxelSize, 0.00052f, /*tolerance=*/1e-4);

        PointDataGrid::Ptr grid = Local::genPointsGrid(voxelSize, position);
        const Index64 pointsPerVoxel = math::Round(2000.0f / static_cast<float>(grid->activeVoxelCount()));
        CPPUNIT_ASSERT_EQUAL(pointsPerVoxel, Index64(1));
    }

    // random position generation within three bounds of varying size.
    // This test distributes 1000 points within a 1x1x1 box centered at (0.5,0.5,0,5)
    // another 1000 points within a separate 10x10x10 box centered at (15,15,15) and
    // a final 1000 points within a separate 50x50x50 box centered at (75,75,75)
    // Points are randomly positioned however can be defined as having a stochastic
    // distribution. Tests that sparsity between these data sets causes no issues as
    // well as autoVoxelSize producing a good average result

    {
        position.resize(3000);
        AttributeWrapper<Vec3f>::Handle positionHandle(position);
        openvdb::math::Random01 randNumber(0);

        // positions between 0 and 1

        for(size_t i = 0; i < 1000; ++ i) {
            const Vec3f pos(randNumber());
            positionHandle.set(i, 0, pos);
        }

        // positions between 10 and 20

        for(size_t i = 1000; i < 2000; ++ i) {
            const Vec3f pos((randNumber() * 10.0f) + 10.0f);
            positionHandle.set(i, 0, pos);
        }

        // positions between 50 and 100

        for(size_t i = 2000; i < 3000; ++ i) {
            const Vec3f pos((randNumber() * 50.0f) + 50.0f);
            positionHandle.set(i, 0, pos);
        }

        float voxelSize = autoVoxelSize(position, /*points per voxel*/10);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(voxelSize, 0.24758f, /*tolerance=*/1e-4);

        PointDataGrid::Ptr grid = Local::genPointsGrid(voxelSize, position);
        Index64 pointsPerVoxel = math::Round(3000.0f/ static_cast<float>(grid->activeVoxelCount()));
        CPPUNIT_ASSERT(math::isApproxEqual(pointsPerVoxel, Index64(10), Index64(2)));

        voxelSize = autoVoxelSize(position, /*points per voxel*/1);
        CPPUNIT_ASSERT_DOUBLES_EQUAL(voxelSize, 0.00231f, /*tolerance=*/1e-4);

        grid = Local::genPointsGrid(voxelSize, position);
        pointsPerVoxel = math::Round(3000.0f/ static_cast<float>(grid->activeVoxelCount()));
        CPPUNIT_ASSERT_EQUAL(pointsPerVoxel, Index64(1));
    }

    // Generate a sphere
    // NOTE: The sphere does NOT provide uniform distribution

    const unsigned long count(40000);

    position.resize(0);

    AttributeWrapper<int> xyz(1);
    AttributeWrapper<int> id(1);
    AttributeWrapper<float> uniform(1);
    AttributeWrapper<openvdb::Name> string(1);
    GroupWrapper group;

    genPoints(count, /*scale=*/ 100.0, /*stride=*/false, position, xyz, id, uniform, string, group);

    CPPUNIT_ASSERT_EQUAL(position.size(), count);
    CPPUNIT_ASSERT_EQUAL(id.size(), count);
    CPPUNIT_ASSERT_EQUAL(uniform.size(), count);
    CPPUNIT_ASSERT_EQUAL(string.size(), count);
    CPPUNIT_ASSERT_EQUAL(group.size(), count);

    // test a distributed point set around a sphere

    {
        const float voxelSize = autoVoxelSize(position, /*points per voxel*/2);

        CPPUNIT_ASSERT_DOUBLES_EQUAL(voxelSize, 2.6275f, /*tolerance=*/1e-4);

        PointDataGrid::Ptr grid = Local::genPointsGrid(voxelSize, position);
        const Index64 pointsPerVoxel = count / grid->activeVoxelCount();
        CPPUNIT_ASSERT_EQUAL(pointsPerVoxel, Index64(2));
    }

    // test with given target transforms

    {
        // test that a different scale doesn't change the result

        openvdb::math::Transform::Ptr transform1(openvdb::math::Transform::createLinearTransform(0.33));
        openvdb::math::Transform::Ptr transform2(openvdb::math::Transform::createLinearTransform(0.87));

        math::UniformScaleMap::ConstPtr scaleMap1 = transform1->constMap<math::UniformScaleMap>();
        math::UniformScaleMap::ConstPtr scaleMap2 = transform2->constMap<math::UniformScaleMap>();
        CPPUNIT_ASSERT(scaleMap1.get());
        CPPUNIT_ASSERT(scaleMap2.get());

        math::AffineMap::ConstPtr affineMap1 = scaleMap1->getAffineMap();
        math::AffineMap::ConstPtr affineMap2 = scaleMap2->getAffineMap();

        float voxelSize1 = autoVoxelSize(position, /*points per voxel*/2, max, min, affineMap1->getMat4());
        float voxelSize2 = autoVoxelSize(position, /*points per voxel*/2, max, min, affineMap2->getMat4());
        CPPUNIT_ASSERT_EQUAL(voxelSize1, voxelSize2);

        // test that applying a rotation roughly calculates to the same result for this example
        // NOTE: distribution is not uniform

        // Rotate by 45 degrees in X, Y, Z

        transform1->postRotate(M_PI / 4.0, math::X_AXIS);
        transform1->postRotate(M_PI / 4.0, math::Y_AXIS);
        transform1->postRotate(M_PI / 4.0, math::Z_AXIS);

        affineMap1 = transform1->constMap<math::AffineMap>();
        CPPUNIT_ASSERT(affineMap1.get());

        float voxelSize3 = autoVoxelSize(position, /*points per voxel*/2, max, min, affineMap1->getMat4());
        CPPUNIT_ASSERT_DOUBLES_EQUAL(voxelSize1, voxelSize3, 0.1);

        // test that applying a translation roughly calculates to the same result for this example

        transform1->postTranslate(Vec3d(-5.0f, 3.3f, 20.1f));
        affineMap1 = transform1->constMap<math::AffineMap>();
        CPPUNIT_ASSERT(affineMap1.get());

        float voxelSize4 = autoVoxelSize(position, /*points per voxel*/2, max, min, affineMap1->getMat4());
        CPPUNIT_ASSERT_DOUBLES_EQUAL(voxelSize1, voxelSize4, 0.1);
    }
}

// Copyright (c) 2015-2016 Double Negative Visual Effects
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
