package com.example.controlserver.Repositories;

import com.example.controlserver.Models.UGV;
import org.springframework.data.mongodb.repository.MongoRepository;
import org.springframework.stereotype.Repository;

@Repository
public interface UGVRepository extends MongoRepository<UGV, String> {

}
