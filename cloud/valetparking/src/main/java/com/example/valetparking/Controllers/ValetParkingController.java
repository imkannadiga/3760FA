package com.example.valetparking.Controllers;

import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import com.example.valetparking.Helpers.ControlHelper;
import com.example.valetparking.Helpers.DropOffSpotHelper;
import com.example.valetparking.Helpers.ParkingHelper;
import com.example.valetparking.Helpers.Phase;
import com.example.valetparking.Helpers.SpotType;
import com.example.valetparking.Helpers.Task;
import com.example.valetparking.Models.DropOffSpot;
import com.example.valetparking.Models.Requests;
import com.example.valetparking.Models.UGVStatus;
import com.example.valetparking.Repositories.RequestRepository;
import com.example.valetparking.Repositories.UGVStatusRepository;
import com.google.gson.JsonObject;

import java.util.List;
import java.util.Map;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;

@RestController
@RequestMapping("/api/valetparking")
public class ValetParkingController {

    @Autowired
    private ControlHelper controlHelper;

    @Autowired
    private DropOffSpotHelper dropOffSpotHelper;

    @Autowired
    private RequestRepository requestRepository;

    @Autowired
    private UGVStatusRepository ugvStatusRepository;

    @Autowired
    private ParkingHelper parkingHelper;

    @PostMapping("/park")
    public ResponseEntity<Object> parkUGV(@RequestBody Map<String, Object> payload) {

        String ugvId = (String) payload.get("ugvID");

        try {
            if (!controlHelper.checkIfUGVIsValid(ugvId)) {
                return ResponseEntity.badRequest().body("UGV ID invalid or UGV not online");
            }

            if (!controlHelper.checkIfUGVIsParkable(ugvId)) {
                return ResponseEntity.badRequest().body("UGV is not parkable at the moment");
            }

        } catch (Exception e) {
            e.printStackTrace();
            return ResponseEntity.internalServerError().body("Internal server error......"+e.getMessage());
        }

        // Search for available drop off location and block it
        DropOffSpot spot = dropOffSpotHelper.getAndBlockAvailableSpot(ugvId, SpotType.DROPOFF);

        if(spot==null) {
            return ResponseEntity.status(422).body("No drop off spot available");
        }

        // Send ugv ID and drop off location with callback url to the control server
        String navigationId = controlHelper.sendRequestToControlServer(ugvId, spot.getCoordinates());

        // Create a requests entity
        Requests request = new Requests(ugvId, Phase.DROP_OFF_LOCATION, Task.PARK, navigationId);
        requestRepository.save(request);

        // Create or update a UGV status entry
        // try to fetch from database if record already exists
        List<UGVStatus> _ugvStatus = ugvStatusRepository.findByUgvId(ugvId);
        UGVStatus ugvStatus = _ugvStatus.isEmpty() ? new UGVStatus() : _ugvStatus.get(0);
        ugvStatus.setCurrentPhase(Phase.DROP_OFF_LOCATION);
        ugvStatus.setUgvId(ugvId);
        ugvStatusRepository.save(ugvStatus);

        return ResponseEntity.ok().body("Started parking routine....");
    }

    @PostMapping("/retrieve")
    public ResponseEntity<Object> retrieveUGV(@RequestBody Map<String, Object> payload) {

        String ugvId = (String) payload.get("ugvID");

        try {
            if (!controlHelper.checkIfUGVIsValid(ugvId)) {
                return ResponseEntity.badRequest().body("UGV ID invalid or UGV not online");
            }

            if (!controlHelper.checkIfUGVIsRetrieveable(ugvId)) {
                return ResponseEntity.badRequest().body("UGV is not retrieveable at the moment");
            }

        } catch (Exception e) {
            e.printStackTrace();
            return ResponseEntity.internalServerError().body("Internal server error......please contact developer");
        }

        // Search for available drop off location and block it
        DropOffSpot spot = dropOffSpotHelper.getAndBlockAvailableSpot(ugvId, SpotType.PICKUP);

        if(spot==null) {
            return ResponseEntity.status(422).body("No drop off spot available");
        }

        // release parking spot
        Requests previousReq = requestRepository.findByUgvId(ugvId).get(0);
        parkingHelper.releaseParkingSpot(previousReq.getParkingSpotId());

        // Send ugv ID and drop off location with callback url to the control server
        String navigationId = controlHelper.sendRequestToControlServer(ugvId, spot.getCoordinates());

        // Create a requests entity
        Requests request = new Requests(ugvId, Phase.DROP_OFF_LOCATION, Task.RETRIEVE, navigationId);
        requestRepository.save(request);

        // Create or update a UGV status entry
        // try to fetch from database if record already exists
        List<UGVStatus> _ugvStatus = ugvStatusRepository.findByUgvId(ugvId);
        UGVStatus ugvStatus = _ugvStatus.isEmpty() ? new UGVStatus() : _ugvStatus.get(0);
        ugvStatus.setCurrentPhase(Phase.DROP_OFF_LOCATION);
        ugvStatus.setUgvId(ugvId);
        ugvStatusRepository.save(ugvStatus);

        return ResponseEntity.ok().body("Started retrieve routine....");
    }

    @PostMapping("/action-complete")
    public ResponseEntity<Object> handleActionComplete(@RequestBody Map<String, Object> payload) {

        // get navigation id from payload
        String navigationId = (String) payload.get("request_id");

        // get curresponding requests entity from database
        List<Requests> requests = requestRepository.findByCurrentJobId(navigationId);
        Requests req = requests.get(0);

        req.setStepTwoPending(true);

        requestRepository.save(req);

        return ResponseEntity.ok().build();
    }
}