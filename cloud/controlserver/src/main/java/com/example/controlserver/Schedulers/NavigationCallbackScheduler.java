package com.example.controlserver.Schedulers;

import java.util.List;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.scheduling.annotation.Scheduled;
import org.springframework.stereotype.Component;

import com.example.controlserver.Models.NavigationRequest;
import com.example.controlserver.Services.NavigationRequestService;

@Component
public class NavigationCallbackScheduler {

    @Autowired
    NavigationRequestService navigationRequestService;

    private static final Logger logger = LoggerFactory.getLogger(NavigationCallbackScheduler.class);
    
    @Scheduled(fixedDelay = 10000)
    public void completedNavigationCallbacks() {
        logger.info("Started navigation callback scheduler");
        // Get all completed navigation requests
        List<NavigationRequest> completedRequests = navigationRequestService.getCompletedJobs();

        for(NavigationRequest req : completedRequests) {
            navigationRequestService.generateCallBack(req.getId(), req.getCallbackURL());
        }
    }
}
