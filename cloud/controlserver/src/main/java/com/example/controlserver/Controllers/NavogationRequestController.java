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

import com.example.controlserver.Misc.JobStatus;
import com.example.controlserver.Models.NavigationRequest;
import com.example.controlserver.Services.NavigationRequestService;

@RestController
@RequestMapping("/api/navigation-request")
public class NavogationRequestController {
    
    @Autowired
    private NavigationRequestService navigationRequestService;
    
    @PostMapping
    public ResponseEntity<NavigationRequest> createNavigationRequest(@RequestBody NavigationRequest navigationRequest) {
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
    public ResponseEntity<NavigationRequest> updateNavigationRequest(@PathVariable String id, @RequestBody NavigationRequest navDetails) {
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
