package com.example.valetparking.Helpers;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Service;
import org.springframework.web.client.RestTemplate;

import com.example.valetparking.Models.UGVStatus;
import com.example.valetparking.Repositories.RequestRepository;
import com.example.valetparking.Repositories.UGVStatusRepository;
import com.fasterxml.jackson.databind.ObjectMapper;

@Service
public class ControlHelper {

    @Autowired
    private RestTemplate restTemplate;

    @Autowired
    private RequestRepository requestRepository;

    @Autowired 
    private UGVStatusRepository ugvStatusRepository;

    private final ObjectMapper objectMapper = new ObjectMapper();
    
    private final String CONTROL_SERVER_ADDRESS = "http://controlserver:10001/api";

    private final String CALLBACK_URL = "http://valetparking:9001/api/valetparking/action-complete";

    public String sendRequestToControlServer(String ugvId, Map<String, Object> targetPose) {

        // Generate payload to send to server
        Map<String, Object> payload = new HashMap<>();
        payload.put("ugvId", ugvId);
        payload.put("targetPose", targetPose);
        payload.put("callbackURL", CALLBACK_URL);

        // Send message to control server
        ResponseEntity<Map> resp = restTemplate.postForEntity(CONTROL_SERVER_ADDRESS+"/api/navigation-request", payload, Map.class);

        return (String) resp.getBody().get("request_id");

    }

    public boolean checkIfUGVIsValid(String ugvId) throws Exception{

        String response_str = (String) restTemplate.getForObject(CONTROL_SERVER_ADDRESS+"/ugv", String.class);

        Map<String, Object> json_resp = objectMapper.readValue(response_str, Map.class);

        // check to see if recieved response is empty or invalid
        if(json_resp.isEmpty() || !json_resp.containsKey("id")) return false;

        // check to see if UGV is online
        if(json_resp.get("status")!="ONLINE") return false;

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

        Optional<UGVStatus> ugvStatus = ugvStatusRepository.findById(ugvId);

        // If there is no status entry, the UGV has not been parked and hence, cannot be retrieved
        if(!ugvStatus.isPresent()) return false;

        // If the UGV is not in the PARKED phase and is already present in the status records, it cannot be retrieved
        if(ugvStatus.get().getCurrentPhase() != Phase.PARKED) return false;

        return true;

    }

}
