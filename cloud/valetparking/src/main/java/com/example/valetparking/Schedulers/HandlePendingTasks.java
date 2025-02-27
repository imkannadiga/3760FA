package com.example.valetparking.Schedulers;

import org.springframework.scheduling.annotation.Scheduled;
import org.springframework.stereotype.Component;

@Component
public class HandlePendingTasks {

    @Scheduled(fixedRate = 1000)
    public void handleNextStep() {
        
    }
    
}
