#pragma once

enum class HazardType : int { NONE = 0, DUST_PLUME = 1, FIBROUS_HAZARD, TOXIC_GAS };

enum class TaskType: int { NONE = 0, INITIAL_INSPECTION = 1, DUST_PLUME_ACTIONS, ASBESTOS_ANALYSIS};

enum class CommandType: int {NONE = 0, APPROVED = 1, CANCELLED };

enum class Edge: int { NORTH = 1, SOUTH, EAST, WEST };

enum class OrbitMode: int { NONE = 0, RADIAL_IN = 1, TANGENT_ALIGN, RADIAL_OUT, ORBIT};

enum class SeverityLevel: int { NONE = 0, LOW = 1, MODERATE, HIGH, VERY_HIGH, SEVERE};

enum class OrbitHazardManagerPluginType {Nav2 = 1, Twist};