package com.example.mapserver.Controllers;

import com.example.mapserver.Models.MapRecord;
import com.example.mapserver.Repositories.MapRecordRepository;
import org.springframework.web.bind.annotation.*;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.ResponseEntity;

import java.util.List;
import java.util.Map;

@CrossOrigin(origins = "*")
@RestController
@RequestMapping("/api/costmap")
public class CostmapController {

    @Autowired
    private MapRecordRepository mapRecordRepository;

    private static final int MAX_RECORDS = 10;

    @PostMapping("/upload_map")
    public ResponseEntity<?> uploadMap(@RequestBody Map<String, Object> mapData) {
        try {
            long timestamp = System.currentTimeMillis();

            // Extract map info and origin
            Map<String, Object> mapInfo = (Map<String, Object>) ((Map<String, Object>) mapData.get("map")).get("info");
            Map<String, Object> mapOrigin = (Map<String, Object>) mapInfo.get("origin");
            // System.out.println("MAP_ORIGIN_X ::: "+(double) mapOrigin.get("x"));
            List<Integer> mapDataList = (List<Integer>) ((Map<String, Object>) mapData.get("map")).get("data");

            // Extract global costmap info and origin
            Map<String, Object> globalInfo = (Map<String, Object>) ((Map<String, Object>) mapData.get("global_costmap")).get("info");
            Map<String, Object> globalOrigin = (Map<String, Object>) globalInfo.get("origin");
            List<Integer> globalDataList = (List<Integer>) ((Map<String, Object>) mapData.get("global_costmap")).get("data");

            // Extract local costmap info and origin
            Map<String, Object> localInfo = (Map<String, Object>) ((Map<String, Object>) mapData.get("local_costmap")).get("info");
            Map<String, Object> localOrigin = (Map<String, Object>) localInfo.get("origin");
            List<Integer> localDataList = (List<Integer>) ((Map<String, Object>) mapData.get("local_costmap")).get("data");

            // Extract transform data
            Map<String, Object> transform = (Map<String, Object>) mapData.get("transform");
            Map<String, Object> translation = (Map<String, Object>) transform.get("translation");
            Map<String, Object> rotation = (Map<String, Object>) transform.get("rotation");

            double mapOriginX = (double) mapOrigin.get("x");
            double mapOriginY = (double) mapOrigin.get("y");
            double mapOriginZ = (double) mapOrigin.get("z");

            // System.out.println("MapOriginX ::: "+mapOriginX);
            // System.out.println("MapOriginY ::: "+mapOriginY);
            // System.out.println("MapOriginZ ::: "+mapOriginZ);
            
            double globalMapOriginX = (double) globalOrigin.get("x");
            double globalMapOriginY = (double) globalOrigin.get("y");
            double globalMapOriginZ = (double) globalOrigin.get("z");
            
            double localMapOriginX = (double) localOrigin.get("x");
            double localMapOriginY = (double) localOrigin.get("y");
            double localMapOriginZ = (double) localOrigin.get("z");
            

            // Create a new MapRecord with origin data
            MapRecord newRecord = new MapRecord(
                timestamp,
                (int) mapInfo.get("width"), (int) mapInfo.get("height"), (double) mapInfo.get("resolution"), mapDataList,
                mapOriginX, mapOriginY, mapOriginZ,
                (int) globalInfo.get("width"), (int) globalInfo.get("height"), (double) globalInfo.get("resolution"), globalDataList,
                globalMapOriginX, globalMapOriginY, globalMapOriginZ,
                (int) localInfo.get("width"), (int) localInfo.get("height"), (double) localInfo.get("resolution"), localDataList,
                localMapOriginX, localMapOriginY, localMapOriginZ,
                (double) translation.get("x"), (double) translation.get("y"), (double) translation.get("z"),
                (double) rotation.get("x"), (double) rotation.get("y"), (double) rotation.get("z"), (double) rotation.get("w")
            );

            // Save the new record and clean up old records
            mapRecordRepository.save(newRecord);
            cleanOldRecords();

            // System.out.println("Map uploaded successfully");
            
            return ResponseEntity.ok(Map.of("message", "Maps and transform uploaded successfully", "timestamp", timestamp));
        } catch (Exception e) {
            e.printStackTrace();
            return ResponseEntity.status(500).body(Map.of("error", "Failed to save maps and transform"));
        }
    }

    @GetMapping("/download_map")
    public ResponseEntity<?> downloadMap() {
        List<MapRecord> records = mapRecordRepository.findAllByOrderByTimestampAsc();
        if (records.isEmpty()) {
            return ResponseEntity.ok(Map.of("message", "No map available"));
        }

        MapRecord latestRecord = records.get(records.size() - 1);

        // Build the response including origin data
        Map<String, Object> response = Map.of(
            "map", Map.of(
                "info", latestRecord.getMapInfo(),
                "data", latestRecord.getMapData()
            ),
            "global_costmap", Map.of(
                "info", latestRecord.getGlobalCostmapInfo(),
                "data", latestRecord.getGlobalCostmapData()
            ),
            "local_costmap", Map.of(
                "info", latestRecord.getLocalCostmapInfo(),
                "data", latestRecord.getLocalCostmapData()
            ),
            "transform", latestRecord.getTransformData()
        );

        return ResponseEntity.ok(response);
    }

    private void cleanOldRecords() {
        List<MapRecord> records = mapRecordRepository.findAllByOrderByTimestampAsc();
        while (records.size() > MAX_RECORDS) {
            mapRecordRepository.delete(records.get(0));
            records.remove(0);
        }
    }
}