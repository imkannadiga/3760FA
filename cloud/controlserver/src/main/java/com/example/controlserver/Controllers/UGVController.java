package com.example.controlserver.Controllers;

import com.example.controlserver.Models.UGV;
import com.example.controlserver.Services.UGVService;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.List;
import java.util.Map;



@RestController
@RequestMapping("/api/ugv")
public class UGVController {

    @Autowired
    private UGVService ugvService;

    @PostMapping
    public ResponseEntity<UGV> createUGV(@RequestBody UGV ugv) {
        UGV createdUGV = ugvService.saveUGV(ugv);
        return ResponseEntity.ok(createdUGV);
    }

    @GetMapping
    public ResponseEntity<List<UGV>> getAllUGVs() {
        List<UGV> ugvs = ugvService.getAllUGVs();
        return ResponseEntity.ok(ugvs);
    }

    @GetMapping("/{id}")
    public ResponseEntity<UGV> getUGVById(@PathVariable String id) {
        UGV ugv = ugvService.getUGVById(id);
        return ResponseEntity.ok(ugv);
    }

    @PutMapping("/{id}")
    public ResponseEntity<UGV> updateUGV(@PathVariable String id, @RequestBody UGV ugvDetails) {
        UGV updatedUGV = ugvService.updateUGV(id, ugvDetails);
        return ResponseEntity.ok(updatedUGV);
    }

    @DeleteMapping("/{id}")
    public ResponseEntity<Void> deleteUGV(@PathVariable String id) {
        ugvService.deleteUGV(id);
        return ResponseEntity.noContent().build();
    }

    @PutMapping("/{id}/registerSession")
    public ResponseEntity<?> setSessionId(@PathVariable String id, @RequestBody Map<String, String> sessionId) {
        try {
            ugvService.getUGVById(id);
        } catch (Exception e) {
            // System.out.println("UGV not found");
            return ResponseEntity.status(404).body("Invalid UGV ID");
        }

        return ResponseEntity.ok(ugvService.setUGVSessionID(id, sessionId.get("sessionId")));
    }

    /*
    
    @PostMapping("/{id}/navigateToGoal")
    public ResponseEntity<String> postGoalPose(@PathVariable String id, @RequestBody Map<String, Object> body) {

        UGV ugv = ugvService.getUGVById(id);

        String sockerSessionId = ugv.getSessionId();

        

        // System.out.println("coordinates "+body.get("point").toString());

        ObjectMapper objectMapper = new ObjectMapper();


        try {
            String mappedCoordinates = objectMapper.writeValueAsString(body.get("point"));

            // System.out.println("MappedCoordinates :::: "+mappedCoordinates);

            socketConnectionHandler.sendMessageToClient(sockerSessionId, new TextMessage(mappedCoordinates));
        } catch (Exception e) {
            e.printStackTrace();
            return ResponseEntity.status(500).body("Internal server error ::: "+e.getMessage());
        }
        
        return ResponseEntity.status(200).body("Sent message successfully");
    }
    
    */
}
