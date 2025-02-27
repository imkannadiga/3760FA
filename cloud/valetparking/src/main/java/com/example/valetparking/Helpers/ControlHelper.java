package com.example.valetparking.Helpers;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Service;
import org.springframework.web.client.RestTemplate;

import com.example.valetparking.Models.UGVStatus;
import com.example.valetparking.Repositories.UGVStatusRepository;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

@Service
public class ControlHelper {

    @Autowired
    private RestTemplate restTemplate;

    @Autowired 
    private UGVStatusRepository ugvStatusRepository;
    
    private final String CONTROL_SERVER_ADDRESS = "http://controlserver:10001/api";

    private final String CALLBACK_URL = "http://valetparking:9000/api/valetparking/action-complete";

    public String sendRequestToControlServer(String ugvId, Map<String, Object> targetPose) {

        // Generate payload to send to server
        Map<String, Object> payload = new HashMap<>();
        payload.put("ugvId", ugvId);
        payload.put("targetPose", targetPose);
        payload.put("callbackURL", CALLBACK_URL);

        // Send message to control server
        ResponseEntity<Map> resp = restTemplate.postForEntity(CONTROL_SERVER_ADDRESS+"/navigation-request", payload, Map.class);

        return (String) resp.getBody().get("request_id");

    }

    public boolean checkIfUGVIsValid(String ugvId) throws Exception {

        String resp_str = restTemplate.getForObject(CONTROL_SERVER_ADDRESS+"/ugv", String.class);

        JsonObject resp = new JsonParser().parse(resp_str).getAsJsonArray().get(0).getAsJsonObject();

        System.out.println(resp.toString());

        // check to see if recieved response is empty or invalid
        if(resp.isJsonNull() || !resp.has("id")) return false;

        System.out.println("Json not null and has ID");

        // check to see if UGV is online
        if(!resp.get("status").getAsString().equals("ONLINE")) return false;

        return true;
    }   

    public boolean checkIfUGVIsParkable(String ugvId) {

        Optional<UGVStatus> ugvStatus = ugvStatusRepository.findById(ugvId);

        //If there is no status entry, the UGV has not been parked or retreived even once. Hence assuming it's available
        if(!ugvStatus.isPresent()) return true;

        //If the UGV is not in the DRIVE_AWAY_LOCATION and is already present in the status records, it cannot be parked again
        if(ugvStatus.get().getCurrentPhase() != Phase.DRIVE_AWAY_LOCATION) return false;

        return true;

    }

    public boolean checkIfUGVIsRetrieveable(String ugvId) {

        List<UGVStatus> _ugvStatus = ugvStatusRepository.findByUgvId(ugvId);
        UGVStatus ugvStatus = _ugvStatus.isEmpty() ? null : _ugvStatus.get(0);

        System.out.println(_ugvStatus);

        // If there is no status entry, the UGV has not been parked and hence, cannot be retrieved
        if(ugvStatus==null) return false;

        System.out.println("Post null check");

        // If the UGV is not in the PARKED phase and is already present in the status records, it cannot be retrieved
        if(ugvStatus.getCurrentPhase() != Phase.PARKED) return false;

        System.out.println("Post parking check");

        return true;

    }

}
