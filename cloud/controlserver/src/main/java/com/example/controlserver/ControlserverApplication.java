package com.example.controlserver;

import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.scheduling.annotation.EnableScheduling;

@SpringBootApplication
@EnableScheduling
public class ControlserverApplication {

	public static void main(String[] args) {
		SpringApplication.run(ControlserverApplication.class, args);
	}

}
