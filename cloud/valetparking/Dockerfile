# Use Maven to build the application
FROM maven:3.9-eclipse-temurin-17 AS builder

# Set the working directory inside the container
WORKDIR /app

# Copy the project files into the container
COPY . .

# Build the Spring Boot application
RUN mvn clean package -DskipTests

# Use a lightweight JDK image to run the built JAR
FROM eclipse-temurin:17-jdk-alpine

# Set the working directory inside the container
WORKDIR /app

# Copy the built JAR from the previous stage
COPY --from=builder /app/target/*.jar app.jar

# Run the application
ENTRYPOINT ["java", "-jar", "app.jar"]
