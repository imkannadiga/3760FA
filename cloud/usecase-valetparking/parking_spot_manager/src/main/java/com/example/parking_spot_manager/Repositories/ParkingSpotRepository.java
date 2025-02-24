package com.example.parking_spot_manager.Repositories;

import org.springframework.data.mongodb.repository.MongoRepository;
import org.springframework.stereotype.Repository;

import com.example.parking_spot_manager.Models.ParkingSpot;

@Repository
public interface ParkingSpotRepository extends MongoRepository<ParkingSpot, String>{
    
}
