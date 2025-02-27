package com.example.valetparking.Models;

import org.springframework.data.annotation.Id;
import org.springframework.data.mongodb.core.mapping.Document;

import com.example.valetparking.Helpers.Phase;
import com.example.valetparking.Helpers.Task;

@Document(collection = "valet_parking_requests")
public class Requests {
    
    @Id
    private String id;

    private String ugvId;

    private Phase currentPhase;

    private Task currentTask;

    private String currentJobId;

    private String parkingSpotId;

    private boolean stepTwoPending;

    public Requests(String ugvId, Phase currentPhase, Task currentTask, String currentJobId) {
        this.ugvId = ugvId;
        this.currentPhase = currentPhase;
        this.currentTask = currentTask;
        this.currentJobId = currentJobId;
    }

    public String getUgvId() {
        return ugvId;
    }

    public void setUgvId(String ugvId) {
        this.ugvId = ugvId;
    }

    public Phase getCurrentPhase() {
        return currentPhase;
    }

    public void setCurrentPhase(Phase currentPhase) {
        this.currentPhase = currentPhase;
    }

    public Task getCurrentTask() {
        return currentTask;
    }

    public void setCurrentTask(Task currentTask) {
        this.currentTask = currentTask;
    }

    public String getCurrentJobId() {
        return currentJobId;
    }

    public void setCurrentJobId(String currentJobId) {
        this.currentJobId = currentJobId;
    }

    public void setParkingSpotId(String parkingSpotId) {
        this.parkingSpotId = parkingSpotId;
    }

    public String getParkingSpotId() {
        return this.parkingSpotId;
    }

    public boolean isStepTwoPending() {
        return stepTwoPending;
    }

    public void setStepTwoPending(boolean stepTwoPending) {
        this.stepTwoPending = stepTwoPending;
    }

}
