package com.example.controlserver.Schedulers;

import org.springframework.scheduling.annotation.Scheduled;
import org.springframework.stereotype.Component;

@Component
public class NavigationCallbackScheduler {
    
    @Scheduled(fixedRate = 10000)
    public void completedNavigationCallbacks() {

        // TODO
        // Get all completed navigation requests
        // Get callback URL for each and hit it with the request ID and status completed
        // Update status of request to FINISHED

    }

}
