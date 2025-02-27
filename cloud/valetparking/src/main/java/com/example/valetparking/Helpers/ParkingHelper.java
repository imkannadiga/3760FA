package com.example.valetparking.Helpers;

import java.util.HashMap;
import java.util.Map;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;
import org.springframework.web.client.RestTemplate;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

@Service
public class ParkingHelper {

    @Autowired
    private RestTemplate restTemplate;

    private final String PARKING_SERVER_ADDRESS = "http://parking-spot-manager:9001/api/parking-spot";

    public Map<String, Object> getAndBlockAvailableParkingSpot() {

        String resp_str = restTemplate.getForObject(PARKING_SERVER_ADDRESS+"/get-free-space", String.class);

        JsonObject spot_json = new JsonParser().parse(resp_str).getAsJsonObject();

        if(spot_json.isJsonNull()) return null;

        String spotId = (String) spot_json.get("id").getAsString();

        // block parking spot
        restTemplate.postForEntity(PARKING_SERVER_ADDRESS+"/"+spotId+"/block", null, String.class);
        
        Map<String, Object> spot = new HashMap<>();
        spot.put("id", spot_json.get("id").getAsString());
        spot.put("x",spot_json.get("x").getAsFloat());
        spot.put("y",spot_json.get("y").getAsFloat());
        spot.put("theta",spot_json.get("theta").getAsFloat());

        return spot;

    }

    public void releaseParkingSpot(String parkingSpotId) {
        restTemplate.postForEntity(PARKING_SERVER_ADDRESS+"/"+parkingSpotId+"/release", null, String.class);
        return;
    }
    
}
