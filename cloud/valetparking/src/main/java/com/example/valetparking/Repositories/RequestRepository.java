package com.example.valetparking.Repositories;

import org.springframework.data.mongodb.repository.MongoRepository;

import com.example.valetparking.Models.Requests;
import java.util.List;


public interface RequestRepository extends MongoRepository<Requests, String>{

    List<Requests> findByUgvId(String ugvId);
    List<Requests> findByCurrentJobId(String currentJobId);
    List<Requests> findByStepTwoPending(boolean stepTwoPending);
    
} 
