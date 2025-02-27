package com.example.valetparking.Repositories;

import java.util.List;

import org.springframework.data.mongodb.repository.MongoRepository;

import com.example.valetparking.Helpers.SpotType;
import com.example.valetparking.Models.DropOffSpot;

public interface DropOffSpotRepository extends MongoRepository<DropOffSpot, String>{
    List<DropOffSpot> findByOccupiedAndSpotType(boolean occupied, SpotType spotType);
    List<DropOffSpot> findByUgvId(String ugvId);
}
