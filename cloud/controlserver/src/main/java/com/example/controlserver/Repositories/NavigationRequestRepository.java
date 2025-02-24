package com.example.controlserver.Repositories;

import org.springframework.data.mongodb.repository.MongoRepository;
import org.springframework.stereotype.Repository;

import com.example.controlserver.Models.NavigationRequest;
import java.util.List;
import com.example.controlserver.Misc.JobStatus;


@Repository
public interface NavigationRequestRepository extends MongoRepository<NavigationRequest, String>{

    List<NavigationRequest> findByJobStatus(JobStatus jobStatus);
    
}
