package com.example.controlserver.Services;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpEntity;
import org.springframework.http.HttpHeaders;
import org.springframework.http.HttpMethod;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Service;
import org.springframework.web.client.RestTemplate;

import com.example.controlserver.Misc.JobStatus;
import com.example.controlserver.Models.NavigationRequest;
import com.example.controlserver.Repositories.NavigationRequestRepository;

@Service
public class NavigationRequestService {
    @Autowired
    private NavigationRequestRepository navigationRequestRepository;

    public NavigationRequest saveNavigationRequest(NavigationRequest navRequest) {
        return navigationRequestRepository.save(navRequest);
    }

    public List<NavigationRequest> getAllNavigationRequests() {
        return navigationRequestRepository.findAll();
    }

    public NavigationRequest getNavigationRequestById(String id) {
        return navigationRequestRepository.findById(id).orElseThrow(() -> new RuntimeException("Navigation request not found"));
    }

    public NavigationRequest updateNavigationRequest(String id, NavigationRequest navDetails) {
        NavigationRequest navRequest = this.getNavigationRequestById(id);
        navRequest.setUgv(navDetails.getUgv());
        navRequest.setTargetPose(navDetails.getTargetPose());
        navRequest.setJobStatus(navDetails.getJobStatus());
        navRequest.setCallbackURL(navDetails.getCallbackURL());
        navRequest.setComment(navDetails.getComment());
        return navigationRequestRepository.save(navRequest);
    }

    public void deleteNavigationRequest(String id) {
        navigationRequestRepository.deleteById(id);
    }

    public List<NavigationRequest> getPendingJobs() {
        return navigationRequestRepository.findByJobStatus(JobStatus.CREATED);
    }

    public void generateCallBack(String jobId, String callbackURL) {
        RestTemplate restTemplate = new RestTemplate();
        Map<String, Object> requestBody = new HashMap<>();
        requestBody.put("jobId",jobId);
        requestBody.put("status","completed");
        HttpHeaders headers = new HttpHeaders();
        headers.setContentType(MediaType.APPLICATION_JSON);
        HttpEntity<Object> requestEntity = new HttpEntity<>(requestBody, headers);
        ResponseEntity<String> response = restTemplate.exchange(callbackURL, HttpMethod.POST, requestEntity, String.class);
        NavigationRequest navReq = this.getNavigationRequestById(jobId);
        navReq.setComment(response.toString());
        this.updateNavigationRequest(jobId, navReq);
        return;
    }

}
