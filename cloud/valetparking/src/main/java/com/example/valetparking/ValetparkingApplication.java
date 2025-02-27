package com.example.valetparking;

import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.scheduling.annotation.EnableScheduling;

@SpringBootApplication
@EnableScheduling
public class ValetparkingApplication {

	public static void main(String[] args) {
		SpringApplication.run(ValetparkingApplication.class, args);
	}

}
