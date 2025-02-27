package com.example.valetparking.Helpers;

import java.util.List;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import com.example.valetparking.Models.DropOffSpot;
import com.example.valetparking.Repositories.DropOffSpotRepository;

@Service
public class DropOffSpotHelper {

    @Autowired
    DropOffSpotRepository dropOffSpotRepository;

    public DropOffSpot getAndBlockAvailableSpot(String ugvId, SpotType spotType) {
        List<DropOffSpot> freeSpots = dropOffSpotRepository.findByOccupiedAndSpotType(false, spotType);
        if(freeSpots.isEmpty()) {
            return null;
        }
        DropOffSpot current = freeSpots.get(0);
        current.setOccupied(true);
        current.setUgvId(ugvId);
        dropOffSpotRepository.save(current);
        return current;
    }

    public void releaseSpot(String ugvId) {
        List<DropOffSpot> occupiedSpots = dropOffSpotRepository.findByUgvId(ugvId);
        DropOffSpot current = occupiedSpots.get(0);
        current.setOccupied(false);
        current.setUgvId("");
        dropOffSpotRepository.save(current);
        return;
    }
    
}
