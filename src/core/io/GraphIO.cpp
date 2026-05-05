#include "core/io/GraphIO.h"

#include <QFile>
#include <QIODevice>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QJsonValue>
#include <QString>

#include <cmath>
#include <limits>
#include <string>
#include <unordered_set>
#include <utility>

namespace nav {

namespace {

constexpr double kMinImportance = 0.0;
constexpr double kMaxImportance = 1.0;

std::string pathForError(const QString& path) {
    return path.toStdString();
}

bool isFinite(double value) {
    return std::isfinite(value);
}

bool readRequiredObject(const QJsonObject& parent,
                        const QString& key,
                        QJsonObject& out,
                        const std::string& context,
                        std::string& error) {
    const QJsonValue value = parent.value(key);
    if (!value.isObject()) {
        error = context + ": missing or invalid object '" + key.toStdString() + "'";
        return false;
    }
    out = value.toObject();
    return true;
}

bool readRequiredArray(const QJsonObject& parent,
                       const QString& key,
                       QJsonArray& out,
                       const std::string& context,
                       std::string& error) {
    const QJsonValue value = parent.value(key);
    if (!value.isArray()) {
        error = context + ": missing or invalid array '" + key.toStdString() + "'";
        return false;
    }
    out = value.toArray();
    return true;
}

bool readRequiredNumber(const QJsonObject& object,
                        const QString& key,
                        double& out,
                        const std::string& context,
                        std::string& error) {
    const QJsonValue value = object.value(key);
    if (!value.isDouble()) {
        error = context + ": missing or invalid number '" + key.toStdString() + "'";
        return false;
    }

    out = value.toDouble();
    if (!isFinite(out)) {
        error = context + ": non-finite number '" + key.toStdString() + "'";
        return false;
    }
    return true;
}

bool readRequiredBool(const QJsonObject& object,
                      const QString& key,
                      bool& out,
                      const std::string& context,
                      std::string& error) {
    const QJsonValue value = object.value(key);
    if (!value.isBool()) {
        error = context + ": missing or invalid bool '" + key.toStdString() + "'";
        return false;
    }
    out = value.toBool();
    return true;
}

bool readRequiredString(const QJsonObject& object,
                        const QString& key,
                        std::string& out,
                        const std::string& context,
                        std::string& error) {
    const QJsonValue value = object.value(key);
    if (!value.isString()) {
        error = context + ": missing or invalid string '" + key.toStdString() + "'";
        return false;
    }
    out = value.toString().toStdString();
    return true;
}

template <typename Id>
bool readRequiredId(const QJsonObject& object,
                    const QString& key,
                    Id& out,
                    const std::string& context,
                    std::string& error) {
    double raw = 0.0;
    if (!readRequiredNumber(object, key, raw, context, error)) {
        return false;
    }

    constexpr double maxId = static_cast<double>(std::numeric_limits<Id>::max());
    if (raw < 0.0 || raw > maxId || std::floor(raw) != raw) {
        error = context + ": invalid integer id '" + key.toStdString() + "'";
        return false;
    }

    out = static_cast<Id>(raw);
    return true;
}

QString roadClassToString(RoadClass roadClass) {
    switch (roadClass) {
        case RoadClass::Arterial:  return "arterial";
        case RoadClass::Secondary: return "secondary";
        case RoadClass::Local:
        default:                   return "local";
    }
}

bool roadClassFromString(const std::string& text,
                         RoadClass& out,
                         const std::string& context,
                         std::string& error) {
    if (text == "arterial") {
        out = RoadClass::Arterial;
        return true;
    }
    if (text == "secondary") {
        out = RoadClass::Secondary;
        return true;
    }
    if (text == "local") {
        out = RoadClass::Local;
        return true;
    }

    error = context + ": unknown roadClass '" + text + "'";
    return false;
}

QString displayTierToString(DisplayTier displayTier) {
    switch (displayTier) {
        case DisplayTier::Primary:   return "primary";
        case DisplayTier::Secondary: return "secondary";
        case DisplayTier::Local:     return "local";
        case DisplayTier::Detail:
        default:                     return "detail";
    }
}

bool displayTierFromString(const std::string& text,
                           DisplayTier& out,
                           const std::string& context,
                           std::string& error) {
    if (text == "primary") {
        out = DisplayTier::Primary;
        return true;
    }
    if (text == "secondary") {
        out = DisplayTier::Secondary;
        return true;
    }
    if (text == "local") {
        out = DisplayTier::Local;
        return true;
    }
    if (text == "detail") {
        out = DisplayTier::Detail;
        return true;
    }

    error = context + ": unknown displayTier '" + text + "'";
    return false;
}

QJsonObject pointToJson(const Point2D& position) {
    QJsonObject object;
    object["x"] = position.x;
    object["y"] = position.y;
    return object;
}

QJsonObject nodeToJson(const Node& node) {
    QJsonObject object;
    object["id"] = static_cast<double>(node.getId());
    object["position"] = pointToJson(node.getPosition());
    object["properties"] = QJsonObject();
    return object;
}

QJsonObject edgeToJson(const Edge& edge) {
    QJsonObject object;
    object["id"] = static_cast<double>(edge.getId());
    object["source"] = static_cast<double>(edge.getSource());
    object["target"] = static_cast<double>(edge.getTarget());
    object["length"] = edge.getLength();
    object["capacity"] = edge.getCapacity();
    object["roadClass"] = roadClassToString(edge.getRoadClass());
    object["importance"] = edge.getImportanceScore();
    object["displayTier"] = displayTierToString(edge.getDisplayTier());
    object["oneWay"] = edge.isOneWay();
    object["roadName"] = QString::fromStdString(edge.getRoadName());
    return object;
}

} // namespace

std::string GraphIO::lastError_;

bool GraphIO::save(const std::string& filepath, const Graph& graph,
                   double mapWidth, double mapHeight) {
    const QString qPath = QString::fromStdString(filepath);
    QFile file(qPath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        lastError_ = "Cannot open file for writing: " + pathForError(qPath);
        return false;
    }

    QJsonObject mapObject;
    mapObject["width"] = mapWidth;
    mapObject["height"] = mapHeight;

    QJsonArray nodesArray;
    for (const auto& pair : graph.getNodes()) {
        nodesArray.append(nodeToJson(pair.second));
    }

    QJsonArray edgesArray;
    for (const auto& pair : graph.getEdges()) {
        edgesArray.append(edgeToJson(pair.second));
    }

    QJsonObject root;
    root["map"] = mapObject;
    root["nodes"] = nodesArray;
    root["edges"] = edgesArray;

    const QJsonDocument document(root);
    const QByteArray bytes = document.toJson(QJsonDocument::Indented);
    if (file.write(bytes) != bytes.size()) {
        lastError_ = "Write error while saving to: " + pathForError(qPath);
        return false;
    }

    return true;
}

bool GraphIO::load(const std::string& filepath, MapFileData& outData) {
    const QString qPath = QString::fromStdString(filepath);
    QFile file(qPath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        lastError_ = "Cannot open file for reading: " + pathForError(qPath);
        return false;
    }

    const QByteArray bytes = file.readAll();
    if (bytes.isEmpty()) {
        lastError_ = "Empty file";
        return false;
    }

    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(bytes, &parseError);
    if (parseError.error != QJsonParseError::NoError) {
        lastError_ = "JSON parse error at offset " +
                     std::to_string(parseError.offset) + ": " +
                     parseError.errorString().toStdString();
        return false;
    }
    if (!document.isObject()) {
        lastError_ = "Invalid JSON map file: root must be an object";
        return false;
    }

    const QJsonObject root = document.object();
    std::string error;
    QJsonObject mapObject;
    QJsonArray nodesArray;
    QJsonArray edgesArray;
    if (!readRequiredObject(root, "map", mapObject, "root", error) ||
        !readRequiredArray(root, "nodes", nodesArray, "root", error) ||
        !readRequiredArray(root, "edges", edgesArray, "root", error)) {
        lastError_ = error;
        return false;
    }

    MapFileData parsedData;
    if (!readRequiredNumber(mapObject, "width", parsedData.width, "map", error) ||
        !readRequiredNumber(mapObject, "height", parsedData.height, "map", error)) {
        lastError_ = error;
        return false;
    }
    if (parsedData.width <= 0.0 || parsedData.height <= 0.0) {
        lastError_ = "map: width and height must be positive";
        return false;
    }

    parsedData.graph.reserve(static_cast<size_t>(nodesArray.size()),
                             static_cast<size_t>(edgesArray.size()));

    std::unordered_set<Node::Id> nodeIds;
    for (int i = 0; i < nodesArray.size(); ++i) {
        const std::string context = "nodes[" + std::to_string(i) + "]";
        if (!nodesArray.at(i).isObject()) {
            lastError_ = context + ": entry must be an object";
            return false;
        }

        const QJsonObject nodeObject = nodesArray.at(i).toObject();
        Node::Id id = Node::INVALID_ID;
        QJsonObject positionObject;
        double x = 0.0;
        double y = 0.0;
        if (!readRequiredId(nodeObject, "id", id, context, error) ||
            !readRequiredObject(nodeObject, "position", positionObject, context, error) ||
            !readRequiredNumber(positionObject, "x", x, context + ".position", error) ||
            !readRequiredNumber(positionObject, "y", y, context + ".position", error)) {
            lastError_ = error;
            return false;
        }

        const QJsonValue properties = nodeObject.value("properties");
        if (!properties.isUndefined() && !properties.isObject()) {
            lastError_ = context + ": properties must be an object";
            return false;
        }

        if (!nodeIds.insert(id).second) {
            lastError_ = context + ": duplicate node id " + std::to_string(id);
            return false;
        }

        parsedData.graph.addNodeWithId(id, Point2D(x, y));
    }

    if (parsedData.graph.getNodeCount() == 0) {
        lastError_ = "File contains no nodes";
        return false;
    }

    std::unordered_set<Edge::Id> edgeIds;
    for (int i = 0; i < edgesArray.size(); ++i) {
        const std::string context = "edges[" + std::to_string(i) + "]";
        if (!edgesArray.at(i).isObject()) {
            lastError_ = context + ": entry must be an object";
            return false;
        }

        const QJsonObject edgeObject = edgesArray.at(i).toObject();
        Edge::Id id = Edge::INVALID_ID;
        Node::Id source = Node::INVALID_ID;
        Node::Id target = Node::INVALID_ID;
        double length = 0.0;
        double capacity = 0.0;
        double importance = 0.0;
        bool oneWay = false;
        std::string roadClassText;
        std::string displayTierText;
        std::string roadName;

        if (!readRequiredId(edgeObject, "id", id, context, error) ||
            !readRequiredId(edgeObject, "source", source, context, error) ||
            !readRequiredId(edgeObject, "target", target, context, error) ||
            !readRequiredNumber(edgeObject, "length", length, context, error) ||
            !readRequiredNumber(edgeObject, "capacity", capacity, context, error) ||
            !readRequiredString(edgeObject, "roadClass", roadClassText, context, error) ||
            !readRequiredNumber(edgeObject, "importance", importance, context, error) ||
            !readRequiredString(edgeObject, "displayTier", displayTierText, context, error) ||
            !readRequiredBool(edgeObject, "oneWay", oneWay, context, error) ||
            !readRequiredString(edgeObject, "roadName", roadName, context, error)) {
            lastError_ = error;
            return false;
        }

        if (!edgeIds.insert(id).second) {
            lastError_ = context + ": duplicate edge id " + std::to_string(id);
            return false;
        }
        if (length < 0.0) {
            lastError_ = context + ": length must be non-negative";
            return false;
        }
        if (capacity < 0.0) {
            lastError_ = context + ": capacity must be non-negative";
            return false;
        }
        if (importance < kMinImportance || importance > kMaxImportance) {
            lastError_ = context + ": importance must be in [0, 1]";
            return false;
        }

        RoadClass roadClass = RoadClass::Local;
        DisplayTier displayTier = DisplayTier::Local;
        if (!roadClassFromString(roadClassText, roadClass, context, error) ||
            !displayTierFromString(displayTierText, displayTier, context, error)) {
            lastError_ = error;
            return false;
        }

        if (parsedData.graph.addEdgeWithId(id, source, target, length, capacity, roadClass) == Edge::INVALID_ID) {
            lastError_ = context + ": edge references missing node";
            return false;
        }

        Edge* edge = parsedData.graph.getEdge(id);
        if (!edge) {
            lastError_ = context + ": edge creation failed";
            return false;
        }

        edge->setImportanceScore(importance);
        edge->setDisplayTier(displayTier);
        edge->setOneWay(oneWay);
        edge->setRoadName(roadName);
    }

    outData = std::move(parsedData);
    return true;
}

const std::string& GraphIO::getLastError() {
    return lastError_;
}

} // namespace nav
