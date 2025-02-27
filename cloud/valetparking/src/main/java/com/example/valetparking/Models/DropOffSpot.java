package com.example.valetparking.Models;

import java.util.Map;

import org.springframework.data.annotation.Id;
import org.springframework.data.mongodb.core.mapping.Document;

import com.example.valetparking.Helpers.SpotType;

@Document(collection = "drop_off_spots")
public class DropOffSpot {

    @Id
    private String id;

    private Map<String, Object> coordinates;
    
    private boolean occupied;

    private String ugvId;

    private SpotType spotType;

    public Map<String, Object> getCoordinates() {
        return coordinates;
    }

    public void setCoordinates(Map<String, Object> coordinates) {
        this.coordinates = coordinates;
    }

    public boolean isOccupied() {
        return occupied;
    }

    public void setOccupied(boolean occupied) {
        this.occupied = occupied;
    }

    public String getUgvId() {
        return ugvId;
    }

    public void setUgvId(String ugvId) {
        this.ugvId = ugvId;
    }

    public void setSpotType(SpotType spotType) {
        this.spotType = spotType;
    }

    public SpotType getSpotType() {
        return this.spotType;
    }

}
