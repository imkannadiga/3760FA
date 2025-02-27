package com.example.valetparking.Models;

import org.springframework.data.annotation.Id;
import org.springframework.data.mongodb.core.mapping.Document;

import com.example.valetparking.Helpers.Phase;

@Document(collection = "ugv_status")
public class UGVStatus {

    @Id
    private String id;
    
    private String ugvId;

    private Phase currentPhase;

    public UGVStatus() {
        super();
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

}
