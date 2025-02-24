package com.example.mapserver.Models;

import lombok.Data;
import org.springframework.data.annotation.Id;
import org.springframework.data.mongodb.core.mapping.Document;

import java.util.List;
import java.util.Map;

@Data
@Document(collection = "map_records")
public class MapRecord {

    @Id
    private String id;
    private long timestamp;

    // Map
    private int mapWidth;
    private int mapHeight;
    private double mapResolution;
    private List<Integer> mapData;
    private double mapOriginX; // Origin of the map
    private double mapOriginY;
    private double mapOriginZ;

    // Global Costmap
    private int globalCostmapWidth;
    private int globalCostmapHeight;
    private double globalCostmapResolution;
    private List<Integer> globalCostmapData;
    private double globalCostmapOriginX; // Origin of the global costmap
    private double globalCostmapOriginY;
    private double globalCostmapOriginZ;

    // Local Costmap
    private int localCostmapWidth;
    private int localCostmapHeight;
    private double localCostmapResolution;
    private List<Integer> localCostmapData;
    private double localCostmapOriginX; // Origin of the local costmap
    private double localCostmapOriginY;
    private double localCostmapOriginZ;

    // Transform
    private double translationX;
    private double translationY;
    private double translationZ;
    private double rotationX;
    private double rotationY;
    private double rotationZ;
    private double rotationW;

    public MapRecord(long timestamp,
                     int mapWidth, int mapHeight, double mapResolution, List<Integer> mapData,
                     double mapOriginX, double mapOriginY, double mapOriginZ,
                     int globalCostmapWidth, int globalCostmapHeight, double globalCostmapResolution, List<Integer> globalCostmapData,
                     double globalCostmapOriginX, double globalCostmapOriginY, double globalCostmapOriginZ,
                     int localCostmapWidth, int localCostmapHeight, double localCostmapResolution, List<Integer> localCostmapData,
                     double localCostmapOriginX, double localCostmapOriginY, double localCostmapOriginZ,
                     double translationX, double translationY, double translationZ,
                     double rotationX, double rotationY, double rotationZ, double rotationW) {

        this.timestamp = timestamp;

        this.mapWidth = mapWidth;
        this.mapHeight = mapHeight;
        this.mapResolution = mapResolution;
        this.mapData = mapData;
        this.mapOriginX = mapOriginX;
        this.mapOriginY = mapOriginY;
        this.mapOriginZ = mapOriginZ;

        this.globalCostmapWidth = globalCostmapWidth;
        this.globalCostmapHeight = globalCostmapHeight;
        this.globalCostmapResolution = globalCostmapResolution;
        this.globalCostmapData = globalCostmapData;
        this.globalCostmapOriginX = globalCostmapOriginX;
        this.globalCostmapOriginY = globalCostmapOriginY;
        this.globalCostmapOriginZ = globalCostmapOriginZ;

        this.localCostmapWidth = localCostmapWidth;
        this.localCostmapHeight = localCostmapHeight;
        this.localCostmapResolution = localCostmapResolution;
        this.localCostmapData = localCostmapData;
        this.localCostmapOriginX = localCostmapOriginX;
        this.localCostmapOriginY = localCostmapOriginY;
        this.localCostmapOriginZ = localCostmapOriginZ;

        this.translationX = translationX;
        this.translationY = translationY;
        this.translationZ = translationZ;
        this.rotationX = rotationX;
        this.rotationY = rotationY;
        this.rotationZ = rotationZ;
        this.rotationW = rotationW;
    }

    public Map<String, Object> getMapInfo() {
        return Map.of(
            "width", mapWidth,
            "height", mapHeight,
            "resolution", mapResolution,
            "origin", Map.of("x", mapOriginX, "y", mapOriginY, "z", mapOriginZ)
        );
    }

    public Map<String, Object> getGlobalCostmapInfo() {
        return Map.of(
            "width", globalCostmapWidth,
            "height", globalCostmapHeight,
            "resolution", globalCostmapResolution,
            "origin", Map.of("x", globalCostmapOriginX, "y", globalCostmapOriginY, "z", globalCostmapOriginZ)
        );
    }

    public Map<String, Object> getLocalCostmapInfo() {
        return Map.of(
            "width", localCostmapWidth,
            "height", localCostmapHeight,
            "resolution", localCostmapResolution,
            "origin", Map.of("x", localCostmapOriginX, "y", localCostmapOriginY, "z", localCostmapOriginZ)
        );
    }

    public Map<String, Object> getTransformData() {
        return Map.of(
            "translation", Map.of("x", translationX, "y", translationY , "z", translationZ ),
            "rotation", Map.of("x", rotationX, "y", rotationY, "z", rotationZ, "w", rotationW)
        );
    }
}