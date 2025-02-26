package com.example.controlserver.Controllers;

import java.util.List;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.DeleteMapping;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.PutMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import com.example.controlserver.DTOs.CreateNavigationRequestDTO;
import com.example.controlserver.Misc.JobStatus;
import com.example.controlserver.Misc.UGVStatus;
import com.example.controlserver.Models.NavigationRequest;
import com.example.controlserver.Models.UGV;
import com.example.controlserver.Services.NavigationRequestService;
import com.example.controlserver.Services.UGVService;

@RestController
@RequestMapping("/api/navigation-request")
public class NavigationRequestController {

    @Autowired
    private NavigationRequestService navigationRequestService;

    @Autowired
    private UGVService ugvService;

    @PostMapping
    public ResponseEntity<Object> createNavigationRequest(@RequestBody CreateNavigationRequestDTO dto) {
        // Find the UGV by ID
        UGV ugv = ugvService.getUGVById(dto.getUgvId());

        if(ugv == null) {
            return ResponseEntity.badRequest().body("Invalid UGV ID. UGV not found");
        } 

        if(ugv.getStatus() != UGVStatus.ONLINE) {
            return ResponseEntity.badRequest().body("UGV not online");
        }

        // Create NavigationRequest
        NavigationRequest navigationRequest = new NavigationRequest();
        navigationRequest.setUgv(ugv);
        navigationRequest.setTargetPose(dto.getTargetPose());
        navigationRequest.setCallbackURL(dto.getCallbackURL());

        // Save to DB
        NavigationRequest createdNavigationRequest = navigationRequestService.saveNavigationRequest(navigationRequest);

        return ResponseEntity.ok(createdNavigationRequest);
    }

    @GetMapping
    public ResponseEntity<List<NavigationRequest>> getAllNavigationRequests() {
        List<NavigationRequest> navReqs = navigationRequestService.getAllNavigationRequests();
        return ResponseEntity.ok(navReqs);
    }

    @GetMapping("/{id}")
    public ResponseEntity<NavigationRequest> getNavigationRequestById(@PathVariable String id) {
        NavigationRequest navRequest = navigationRequestService.getNavigationRequestById(id);
        return ResponseEntity.ok(navRequest);
    }

    @PutMapping("/{id}")
    public ResponseEntity<NavigationRequest> updateNavigationRequest(@PathVariable String id,
            @RequestBody NavigationRequest navDetails) {
        NavigationRequest updatedNavRequest = navigationRequestService.updateNavigationRequest(id, navDetails);
        return ResponseEntity.ok(updatedNavRequest);
    }

    @DeleteMapping("/{id}")
    public ResponseEntity<Void> deleteNavigationRequest(@PathVariable String id) {
        navigationRequestService.deleteNavigationRequest(id);
        return ResponseEntity.noContent().build();
    }

    @PostMapping("/{id}/completed")
    public ResponseEntity<Void> completeNavigationRequest(@RequestBody String id) {

        NavigationRequest navReq = navigationRequestService.getNavigationRequestById(id);
        navReq.setJobStatus(JobStatus.COMPLETED);
        navigationRequestService.generateCallBack(navReq.getId(), navReq.getCallbackURL());
        navigationRequestService.updateNavigationRequest(id, navReq);

        return ResponseEntity.noContent().build();
    }

}
