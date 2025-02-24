package com.example.mapserver.Repositories;

import com.example.mapserver.Models.MapRecord;
import org.springframework.data.mongodb.repository.MongoRepository;
import java.util.List;

public interface MapRecordRepository extends MongoRepository<MapRecord, String> {
    List<MapRecord> findAllByOrderByTimestampAsc();
}
