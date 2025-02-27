package com.example.parking_spot_manager.Controllers;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import com.example.parking_spot_manager.Repositories.ParkingSpotRepository;
import com.example.parking_spot_manager.Models.ParkingSpot;

import java.util.List;
import java.util.Optional;


@RestController
@RequestMapping("/api/parking-spot")
public class ParkingSpotController {

    @Autowired
    private ParkingSpotRepository parkingSpotRepository;

    // GET all parking spots
    @GetMapping
    public List<ParkingSpot> getAllParkingSpots() {
        return parkingSpotRepository.findAll();
    }

    // GET a parking spot by ID
    @GetMapping("/{id}")
    public ResponseEntity<ParkingSpot> getParkingSpotById(@PathVariable String id) {
        Optional<ParkingSpot> spot = parkingSpotRepository.findById(id);
        return spot.map(ResponseEntity::ok).orElseGet(() -> ResponseEntity.notFound().build());
    }

    // CREATE a new parking spot
    @PostMapping
    public ResponseEntity<ParkingSpot> createParkingSpot(@RequestBody ParkingSpot parkingSpot) {
        return new ResponseEntity<>(parkingSpotRepository.save(parkingSpot), HttpStatus.CREATED);
    }

    // BLOCK a parking spot
    @PostMapping("/{id}/block")
    public ResponseEntity<String> blockParkingSpot(@PathVariable String id) {
        if (!parkingSpotRepository.existsById(id)) {
            return ResponseEntity.notFound().build();
        }

        Optional<ParkingSpot> spot = parkingSpotRepository.findById(id);
        spot.get().setOccupied(true);
        parkingSpotRepository.save(spot.get());

        return ResponseEntity.status(200).body("Updated successfully");
    }

    // RELEASE a parking spot
    @PostMapping("/{id}/release")
    public ResponseEntity<String> releaseParkingSpot(@PathVariable String id) {
        if (!parkingSpotRepository.existsById(id)) {
            return ResponseEntity.notFound().build();
        }

        Optional<ParkingSpot> spot = parkingSpotRepository.findById(id);
        spot.get().setOccupied(false);
        parkingSpotRepository.save(spot.get());

        return ResponseEntity.status(200).body("Updated successfully");
    }

    // DELETE a parking spot
    @DeleteMapping("/{id}")
    public ResponseEntity<Void> deleteParkingSpot(@PathVariable String id) {
        if (!parkingSpotRepository.existsById(id)) {
            return ResponseEntity.notFound().build();
        }
        parkingSpotRepository.deleteById(id);
        return ResponseEntity.noContent().build();
    }

    // Get first available parking spot
    @GetMapping("/get-free-space")
	public ResponseEntity<?> getFirstFreeSpace() {
		// Retrieve all free parking spots from the repository
		List<ParkingSpot> allSpots = parkingSpotRepository.findByOccupied(false);

		if (allSpots == null || allSpots.isEmpty()) {
			return ResponseEntity.status(HttpStatus.NOT_FOUND).body("No free parking spots available.");
		}

		// Return the first available free spot
		ParkingSpot firstSpot = allSpots.get(0);

		return ResponseEntity.ok(firstSpot);
	}

}
