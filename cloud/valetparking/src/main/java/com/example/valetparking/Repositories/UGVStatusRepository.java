package com.example.valetparking.Repositories;

import org.springframework.data.mongodb.repository.MongoRepository;

import com.example.valetparking.Models.UGVStatus;
import java.util.List;


public interface UGVStatusRepository extends MongoRepository<UGVStatus, String>{
    List<UGVStatus> findByUgvId(String ugvId);
}
