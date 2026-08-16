#include "urdf/UrdfParser.hpp"

#include "tinyxml2.h"

#include <cstdlib>
#include <functional>
#include <set>
#include <sstream>

namespace urdf {
namespace {

using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

[[noreturn]] void fail(const std::string& msg)
{
    throw ParseError(msg);
}

/// URDF numeric attributes are space-separated decimals. Reject anything that does not consume
/// the whole token: "0.1abc" silently parsing as 0.1 is exactly the kind of quiet wrongness this
/// workstream is trying to avoid.
double toDouble(const std::string& tok, const char* context)
{
    try {
        size_t used = 0;
        double v = std::stod(tok, &used);
        while (used < tok.size() && std::isspace(static_cast<unsigned char>(tok[used]))) ++used;
        if (used != tok.size()) fail(std::string(context) + ": not a number: '" + tok + "'");
        return v;
    }
    catch (const ParseError&) {
        throw;
    }
    catch (const std::exception&) {
        fail(std::string(context) + ": not a number: '" + tok + "'");
    }
}

std::vector<double> splitNumbers(const char* text, const char* context)
{
    std::vector<double> out;
    if (!text) return out;
    std::istringstream in(text);
    std::string tok;
    while (in >> tok) out.push_back(toDouble(tok, context));
    return out;
}

Vec3 readVec3(const XMLElement* e, const char* attr, const char* context, Vec3 fallback)
{
    if (!e) return fallback;
    const char* text = e->Attribute(attr);
    if (!text) return fallback;
    auto v = splitNumbers(text, context);
    if (v.size() != 3)
        fail(std::string(context) + ": expected 3 numbers in '" + attr + "', got " + std::to_string(v.size()));
    return Vec3{ v[0], v[1], v[2] };
}

Origin readOrigin(const XMLElement* parent, const char* context)
{
    Origin o;
    if (!parent) return o;
    const XMLElement* e = parent->FirstChildElement("origin");
    if (!e) return o;  // URDF default: identity
    o.xyz = readVec3(e, "xyz", context, Vec3{ 0, 0, 0 });
    o.rpy = readVec3(e, "rpy", context, Vec3{ 0, 0, 0 });
    return o;
}

double readAttrDouble(const XMLElement* e, const char* attr, const char* context, bool required,
                      double fallback = 0.0)
{
    const char* text = e ? e->Attribute(attr) : nullptr;
    if (!text) {
        if (required) fail(std::string(context) + ": missing required attribute '" + attr + "'");
        return fallback;
    }
    return toDouble(text, context);
}

std::string readAttrString(const XMLElement* e, const char* attr, const char* context, bool required)
{
    const char* text = e ? e->Attribute(attr) : nullptr;
    if (!text) {
        if (required) fail(std::string(context) + ": missing required attribute '" + attr + "'");
        return {};
    }
    return text;
}

JointType parseJointType(const std::string& s, const std::string& joint_name)
{
    if (s == "revolute")   return JointType::Revolute;
    if (s == "continuous") return JointType::Continuous;
    if (s == "prismatic")  return JointType::Prismatic;
    if (s == "fixed")      return JointType::Fixed;
    if (s == "floating")   return JointType::Floating;
    if (s == "planar")     return JointType::Planar;
    fail("joint '" + joint_name + "': unknown type '" + s + "'");
}

Geometry parseGeometry(const XMLElement* geom, const std::string& context)
{
    if (!geom) fail(context + ": <geometry> is required");

    Geometry g;
    if (const XMLElement* e = geom->FirstChildElement("box")) {
        g.type = GeometryType::Box;
        g.box_size = readVec3(e, "size", context.c_str(), Vec3{ 0, 0, 0 });
        if (g.box_size.x <= 0 || g.box_size.y <= 0 || g.box_size.z <= 0)
            fail(context + ": box size must be positive in all axes");
    }
    else if (const XMLElement* e = geom->FirstChildElement("cylinder")) {
        g.type = GeometryType::Cylinder;
        g.radius = readAttrDouble(e, "radius", context.c_str(), true);
        g.length = readAttrDouble(e, "length", context.c_str(), true);
        if (g.radius <= 0 || g.length <= 0) fail(context + ": cylinder radius and length must be positive");
    }
    else if (const XMLElement* e = geom->FirstChildElement("sphere")) {
        g.type = GeometryType::Sphere;
        g.radius = readAttrDouble(e, "radius", context.c_str(), true);
        if (g.radius <= 0) fail(context + ": sphere radius must be positive");
    }
    else if (const XMLElement* e = geom->FirstChildElement("mesh")) {
        g.type = GeometryType::Mesh;
        g.mesh_filename = readAttrString(e, "filename", context.c_str(), true);
        g.mesh_scale = readVec3(e, "scale", context.c_str(), Vec3{ 1, 1, 1 });
    }
    else {
        fail(context + ": <geometry> has no box/cylinder/sphere/mesh child");
    }
    return g;
}

void parseLink(const XMLElement* e, Robot& robot)
{
    Link link;
    link.name = readAttrString(e, "name", "link", true);
    if (robot.findLink(link.name) >= 0) fail("duplicate link name '" + link.name + "'");

    const std::string ctx = "link '" + link.name + "'";

    if (const XMLElement* in = e->FirstChildElement("inertial")) {
        const XMLElement* m = in->FirstChildElement("mass");
        if (!m) fail(ctx + ": <inertial> has no <mass>");
        const XMLElement* i = in->FirstChildElement("inertia");
        if (!i) fail(ctx + ": <inertial> has no <inertia>");

        // ⚠ UrdfSim parsed these six terms and then never used them (UrdfParser.cpp:1219 writes
        // the matrix; nothing reads it). Applying them is a correctness goal of this workstream.
        link.inertial = Inertial::fromUrdfTerms(readAttrDouble(i, "ixx", ctx.c_str(), true),
                                                readAttrDouble(i, "ixy", ctx.c_str(), false),
                                                readAttrDouble(i, "ixz", ctx.c_str(), false),
                                                readAttrDouble(i, "iyy", ctx.c_str(), true),
                                                readAttrDouble(i, "iyz", ctx.c_str(), false),
                                                readAttrDouble(i, "izz", ctx.c_str(), true));
        link.inertial.origin = readOrigin(in, ctx.c_str());
        link.inertial.mass   = readAttrDouble(m, "value", ctx.c_str(), true);
        if (link.inertial.mass <= 0) fail(ctx + ": mass must be positive");
        link.has_inertial = true;
    }

    for (const XMLElement* c = e->FirstChildElement("collision"); c;
         c = c->NextSiblingElement("collision")) {
        Collision col;
        const char* n = c->Attribute("name");
        col.name = n ? n : "";
        col.origin = readOrigin(c, ctx.c_str());
        col.geometry = parseGeometry(c->FirstChildElement("geometry"), ctx + " <collision>");
        link.collisions.push_back(col);
    }

    robot.links.push_back(std::move(link));
}

void parseJoint(const XMLElement* e, Robot& robot, const ParseOptions& options)
{
    Joint j;
    j.name = readAttrString(e, "name", "joint", true);
    if (robot.findJoint(j.name) >= 0) fail("duplicate joint name '" + j.name + "'");

    const std::string ctx = "joint '" + j.name + "'";
    j.type = parseJointType(readAttrString(e, "type", ctx.c_str(), true), j.name);

    const XMLElement* p = e->FirstChildElement("parent");
    const XMLElement* c = e->FirstChildElement("child");
    if (!p) fail(ctx + ": missing <parent>");
    if (!c) fail(ctx + ": missing <child>");
    j.parent_link = readAttrString(p, "link", ctx.c_str(), true);
    j.child_link  = readAttrString(c, "link", ctx.c_str(), true);

    j.origin = readOrigin(e, ctx.c_str());

    // URDF default axis is (1,0,0) and applies to revolute/continuous/prismatic/planar.
    j.axis = readVec3(e->FirstChildElement("axis"), "xyz", ctx.c_str(), Vec3{ 1, 0, 0 });

    const bool needs_axis = j.type == JointType::Revolute || j.type == JointType::Continuous ||
                            j.type == JointType::Prismatic || j.type == JointType::Planar;
    if (needs_axis) {
        const double n2 = j.axis.x * j.axis.x + j.axis.y * j.axis.y + j.axis.z * j.axis.z;
        if (n2 < 1e-12) fail(ctx + ": <axis> is zero-length");
    }

    if (const XMLElement* l = e->FirstChildElement("limit")) {
        j.limit.present = true;
        // effort and velocity are required by the URDF spec for revolute/prismatic; lower/upper
        // default to 0 and are meaningless for continuous joints.
        j.limit.lower    = readAttrDouble(l, "lower", ctx.c_str(), false);
        j.limit.upper    = readAttrDouble(l, "upper", ctx.c_str(), false);
        j.limit.effort   = readAttrDouble(l, "effort", ctx.c_str(), false);
        j.limit.velocity = readAttrDouble(l, "velocity", ctx.c_str(), false);
        if (j.type == JointType::Revolute || j.type == JointType::Prismatic) {
            if (j.limit.upper < j.limit.lower)
                fail(ctx + ": limit upper < lower");
        }
    }
    else if (j.type == JointType::Revolute || j.type == JointType::Prismatic) {
        fail(ctx + ": <limit> is required for revolute and prismatic joints");
    }

    if (const XMLElement* d = e->FirstChildElement("dynamics")) {
        j.dynamics.damping  = readAttrDouble(d, "damping", ctx.c_str(), false);
        j.dynamics.friction = readAttrDouble(d, "friction", ctx.c_str(), false);
    }

    // Refuse what UrdfSim accepted and then ignored — unless the caller has explicitly opted in.
    if (const XMLElement* mim = e->FirstChildElement("mimic")) {
        if (!options.ignore_mimic)
            fail(ctx + ": <mimic> is not supported. It is parsed but never applied by UrdfSim; "
                       "accepting it here would silently produce a robot that does not match the "
                       "URDF. Set ParseOptions::ignore_mimic if the coupling is cosmetic and you "
                       "want it loaded as a free joint.");
        j.mimic_ignored = true;
        const char* src = mim->Attribute("joint");
        j.mimic_source_joint = src ? src : "";
    }
    if (e->FirstChildElement("safety_controller"))
        fail(ctx + ": <safety_controller> is not supported (parsed-but-ignored in UrdfSim).");

    robot.joints.push_back(std::move(j));
}

/// Resolve names to indices, verify the tree is a tree, and find the root.
void resolveTree(Robot& robot, const std::string& source)
{
    if (robot.links.empty()) fail(source + ": robot has no links");

    std::set<std::string> children;

    for (size_t ji = 0; ji < robot.joints.size(); ++ji) {
        Joint& j = robot.joints[ji];
        j.parent_index = robot.findLink(j.parent_link);
        j.child_index  = robot.findLink(j.child_link);
        if (j.parent_index < 0)
            fail("joint '" + j.name + "': parent link '" + j.parent_link + "' does not exist");
        if (j.child_index < 0)
            fail("joint '" + j.name + "': child link '" + j.child_link + "' does not exist");
        if (j.parent_index == j.child_index)
            fail("joint '" + j.name + "': parent and child are the same link");

        if (!children.insert(j.child_link).second)
            fail("link '" + j.child_link + "' has more than one parent joint — URDF must be a tree, "
                 "not a closed kinematic chain");

        robot.links[j.child_index].parent_joint = static_cast<int>(ji);
        robot.links[j.parent_index].child_joints.push_back(static_cast<int>(ji));
    }

    for (size_t li = 0; li < robot.links.size(); ++li) {
        if (robot.links[li].parent_joint < 0) {
            if (robot.root_link >= 0)
                fail(source + ": more than one root link ('" + robot.links[robot.root_link].name +
                     "' and '" + robot.links[li].name + "') — the model is disconnected");
            robot.root_link = static_cast<int>(li);
        }
    }
    if (robot.root_link < 0)
        fail(source + ": no root link — the joint graph contains a cycle");
}

} // namespace

int Robot::chainDepth() const
{
    if (root_link < 0) return 0;
    std::function<int(int)> depth = [&](int li) -> int {
        int best = 0;
        for (int ji : links[li].child_joints)
            best = std::max(best, 1 + depth(joints[ji].child_index));
        return best;
    };
    return depth(root_link);
}

Robot parseString(const std::string& xml, const std::string& source_name,
                  const ParseOptions& options)
{
    XMLDocument doc;
    if (doc.Parse(xml.c_str(), xml.size()) != tinyxml2::XML_SUCCESS)
        fail(source_name + ": XML parse error: " + std::string(doc.ErrorStr() ? doc.ErrorStr() : "?"));

    const XMLElement* root = doc.FirstChildElement("robot");
    if (!root) fail(source_name + ": no <robot> root element (xacro must be expanded first)");

    Robot robot;
    const char* name = root->Attribute("name");
    robot.name = name ? name : "unnamed";

    for (const XMLElement* e = root->FirstChildElement("link"); e; e = e->NextSiblingElement("link"))
        parseLink(e, robot);
    for (const XMLElement* e = root->FirstChildElement("joint"); e; e = e->NextSiblingElement("joint"))
        parseJoint(e, robot, options);

    resolveTree(robot, source_name);
    return robot;
}

Robot parseFile(const std::string& path, const ParseOptions& options)
{
    XMLDocument doc;
    if (doc.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS)
        fail(path + ": cannot read file: " + std::string(doc.ErrorStr() ? doc.ErrorStr() : "?"));

    tinyxml2::XMLPrinter printer;
    doc.Print(&printer);
    return parseString(std::string(printer.CStr(), printer.CStrSize() ? printer.CStrSize() - 1 : 0),
                       path, options);
}

} // namespace urdf
