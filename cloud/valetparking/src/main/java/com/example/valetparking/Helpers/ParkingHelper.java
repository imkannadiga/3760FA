package com.example.valetparking.Helpers;

import java.util.Map;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Service;
import org.springframework.web.client.RestTemplate;

@Service
public class ParkingHelper {

    @Autowired
    private RestTemplate restTemplate;

    private final String PARKING_SERVER_ADDRESS = "http://parking-spot_manager:9001/api/parking-spot";

    public Map<String, Object> getAndBlockAvailableParkingSpot() {

        ResponseEntity<Map> spot = restTemplate.getForEntity(PARKING_SERVER_ADDRESS+"/get-free-space", Map.class);

        String spotId = (String) spot.getBody().get("id");

        // block parking spot
        restTemplate.postForEntity(PARKING_SERVER_ADDRESS+"/"+spotId+"/block", null, String.class);
        
        return spot.getBody();

    }

    public void releaseParkingSpot(String parkingSpotId) {
        restTemplate.postForEntity(PARKING_SERVER_ADDRESS+"/"+parkingSpotId+"/release", null, String.class);
        return;
    }
    
}
