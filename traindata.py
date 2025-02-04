import numpy as np
import tensorflow as tf
from sklearn.model_selection import train_test_split
import matplotlib.pyplot as plt

# Simulated Data Generation Function
def generate_data(num_samples):
    # Initialize empty arrays for features and labels
    X = np.zeros((num_samples, 3))  # 3 input features: [distance, speed, IR sensor]
    y = np.zeros((num_samples, 2))  # 2 output labels: [braking force, adjusted speed]
    
    # Generate random data for simulation
    for i in range(num_samples):
        # Simulating sensor data
        distance = np.random.uniform(10, 100)  # Distance to obstacle (in cm)
        speed = np.random.uniform(0, 100)      # Current speed (in km/h)
        ir_sensor = np.random.choice([0, 1])   # IR sensor (0 = no obstacle, 1 = obstacle)
        
        # Define braking force and adjusted speed based on simulated data
        # Braking force depends on the distance to the obstacle and current speed
        if distance < 20:
            braking_force = 1.0  # Max braking
        elif distance < 50:
            braking_force = 0.5  # Medium braking
        else:
            braking_force = 0.0  # No braking
        
        # Adjusted speed: AI will reduce speed based on distance to obstacle
        if ir_sensor == 1:  # Collision detected
            adjusted_speed = 0  # Full stop
        else:
            adjusted_speed = speed - (50 - distance) * 0.2  # Reduce speed based on distance
        
        # Store the generated data into the feature and label arrays
        X[i] = [distance, speed, ir_sensor]
        y[i] = [braking_force, adjusted_speed]

    return X, y

# Generate synthetic training data
num_samples = 1000
X, y = generate_data(num_samples)

# Split data into training and testing sets
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Define the neural network model
model = tf.keras.Sequential([
    tf.keras.layers.Dense(64, input_dim=3, activation='relu'),  # 3 input features: distance, speed, IR
    tf.keras.layers.Dense(64, activation='relu'),
    tf.keras.layers.Dense(2)  # 2 output labels: braking force, adjusted speed
])

# Compile the model
model.compile(optimizer='adam', loss='mean_squared_error', metrics=['mae'])

# Train the model
history = model.fit(X_train, y_train, epochs=100, batch_size=32, validation_data=(X_test, y_test))

# Plot training loss
plt.plot(history.history['loss'], label='Training Loss')
plt.plot(history.history['val_loss'], label='Validation Loss')
plt.title('Training and Validation Loss')
plt.xlabel('Epochs')
plt.ylabel('Loss')
plt.legend()
plt.show()

# Save the model for later use
model.save('ai_braking_model.h5')

# Example of using the model to make predictions
test_data = np.array([[30, 60, 0]])  # Test with distance=30cm, speed=60km/h, no obstacle
predictions = model.predict(test_data)
print("Predicted Braking Force and Adjusted Speed:", predictions)
