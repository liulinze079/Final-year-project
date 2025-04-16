#!/usr/bin/env python3

import os
import numpy as np
import matplotlib.pyplot as plt
import json
import argparse
from itertools import product
import random

def create_directory_structure():
    """Create the directory structure for test scenarios"""
    # Create base directories
    os.makedirs("test_environments/simple", exist_ok=True)
    os.makedirs("test_environments/complex", exist_ok=True)
    os.makedirs("test_environments/real_world", exist_ok=True)
    
    # Create visualization directory
    os.makedirs("test_environments/visualizations", exist_ok=True)
    
    print("Created directory structure for test scenarios")

def generate_obstacle_cluster(center_x, center_y, size, density):
    """
    Generate a cluster of obstacles around a center point
    
    Args:
        center_x: X coordinate of cluster center
        center_y: Y coordinate of cluster center
        size: Size of the cluster (radius)
        density: Density of obstacles (0.0-1.0)
        
    Returns:
        List of obstacle coordinates
    """
    obstacles = []
    
    # Number of obstacles depends on density and size
    num_obstacles = int(size * size * np.pi * density / 4)  # Approximation
    
    for _ in range(num_obstacles):
        # Generate random point within circle
        radius = size * np.sqrt(np.random.random())
        angle = 2 * np.pi * np.random.random()
        
        x = center_x + radius * np.cos(angle)
        y = center_y + radius * np.sin(angle)
        
        obstacles.append((x, y))
    
    return obstacles

def generate_wall_obstacles(start_x, start_y, end_x, end_y, spacing):
    """
    Generate obstacles forming a wall/barrier
    
    Args:
        start_x, start_y: Start point of wall
        end_x, end_y: End point of wall
        spacing: Spacing between obstacles
        
    Returns:
        List of obstacle coordinates
    """
    obstacles = []
    
    # Calculate wall length
    length = np.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
    
    # Calculate number of obstacles
    num_obstacles = int(length / spacing)
    
    # Generate obstacles along the line
    for i in range(num_obstacles):
        t = i / (num_obstacles - 1) if num_obstacles > 1 else 0
        x = start_x + t * (end_x - start_x)
        y = start_y + t * (end_y - start_y)
        obstacles.append((x, y))
    
    return obstacles

def generate_random_obstacles(width, height, num_obstacles, min_distance=0.0):
    """
    Generate randomly positioned obstacles
    
    Args:
        width, height: Map dimensions
        num_obstacles: Number of obstacles to generate
        min_distance: Minimum distance between obstacles
        
    Returns:
        List of obstacle coordinates
    """
    obstacles = []
    
    for _ in range(num_obstacles):
        # Try to place obstacle with minimum distance constraint
        for _ in range(10):  # Max attempts per obstacle
            x = np.random.uniform(-width/2, width/2)
            y = np.random.uniform(-height/2, height/2)
            
            # Check minimum distance to existing obstacles
            valid = True
            for ox, oy in obstacles:
                if np.sqrt((x - ox)**2 + (y - oy)**2) < min_distance:
                    valid = False
                    break
            
            if valid:
                obstacles.append((x, y))
                break
    
    return obstacles

def generate_force_field(width, height, resolution, scenario_type):
    """
    Generate a force field for the scenario
    
    Args:
        width, height: Map dimensions
        resolution: Cell resolution
        scenario_type: Type of scenario for force field pattern
        
    Returns:
        Dictionary with force field data
    """
    force_field = {}
    
    x_coords = np.arange(-width/2, width/2, resolution)
    y_coords = np.arange(-height/2, height/2, resolution)
    
    for x, y in product(x_coords, y_coords):
        # Different patterns based on scenario type
        if scenario_type == "uniform":
            # Uniform current in one direction
            magnitude = 1.0
            direction = np.pi / 4  # 45 degrees
            
        elif scenario_type == "vortex":
            # Vortex/circular pattern
            magnitude = 0.5 + 0.5 * np.sin(np.sqrt(x**2 + y**2) / 50)
            direction = np.arctan2(y, x) + np.pi/2  # Perpendicular to radius
            
        elif scenario_type == "gradient":
            # Gradient flow
            magnitude = 0.2 + 0.8 * (y + height/2) / height
            direction = np.arctan2(1, 0.5 * np.sin(x / 100))
            
        elif scenario_type == "complex":
            # Complex pattern with multiple influences
            magnitude = 0.5 + 0.5 * np.sin(x / 100) * np.cos(y / 100)
            direction = np.arctan2(
                np.sin(y / 100) + 0.2 * x / width,
                np.cos(x / 100) + 0.2 * y / height
            )
            
        else:  # random
            # Random variations
            magnitude = np.random.uniform(0.2, 1.0)
            direction = np.random.uniform(0, 2 * np.pi)
        
        force_field[f"{x:.1f}_{y:.1f}"] = {
            "magnitude": magnitude,
            "direction": direction
        }
    
    return force_field

def generate_environmental_factors(difficulty):
    """
    Generate environmental factors based on difficulty
    
    Args:
        difficulty: Difficulty level (0.0-1.0)
        
    Returns:
        Dictionary with environmental factors
    """
    return {
        "wave_height": difficulty * np.random.uniform(0.5, 2.0),
        "wind_speed": difficulty * np.random.uniform(2.0, 10.0),
        "current_strength": difficulty * np.random.uniform(0.5, 2.0),
        "water_depth": 50.0 - difficulty * np.random.uniform(0.0, 30.0),
        "visibility": 100.0 - difficulty * np.random.uniform(0.0, 70.0)
    }

def generate_shipping_lanes(width, height, num_lanes):
    """
    Generate shipping lanes for navigation
    
    Args:
        width, height: Map dimensions
        num_lanes: Number of shipping lanes
        
    Returns:
        List of shipping lanes, each a list of points
    """
    lanes = []
    
    for _ in range(num_lanes):
        # Generate start and end points for lane
        if np.random.random() < 0.5:
            # Horizontal-ish lane
            x1 = -width/2
            y1 = np.random.uniform(-height/2, height/2)
            x2 = width/2
            y2 = np.random.uniform(-height/2, height/2)
        else:
            # Vertical-ish lane
            x1 = np.random.uniform(-width/2, width/2)
            y1 = -height/2
            x2 = np.random.uniform(-width/2, width/2)
            y2 = height/2
        
        # Add some curve/deviation to the lane
        control_points = [(x1, y1)]
        
        # Add 1-3 control points
        num_controls = np.random.randint(1, 4)
        for i in range(num_controls):
            t = (i + 1) / (num_controls + 1)
            base_x = x1 + t * (x2 - x1)
            base_y = y1 + t * (y2 - y1)
            
            # Add random deviation
            deviation = min(width, height) * 0.1
            offset_x = np.random.uniform(-deviation, deviation)
            offset_y = np.random.uniform(-deviation, deviation)
            
            control_points.append((base_x + offset_x, base_y + offset_y))
        
        # Add end point
        control_points.append((x2, y2))
        
        # Create lane with more points for smooth curve
        lane = []
        for i in range(len(control_points) - 1):
            p1 = control_points[i]
            p2 = control_points[i + 1]
            
            # Add points along segment
            num_points = 10
            for j in range(num_points):
                t = j / num_points
                x = p1[0] + t * (p2[0] - p1[0])
                y = p1[1] + t * (p2[1] - p1[1])
                lane.append((x, y))
        
        lanes.append(lane)
    
    return lanes

def generate_scenario(category, difficulty, width=1000.0, height=1000.0, resolution=10.0):
    """
    Generate a test scenario based on category and difficulty
    
    Args:
        category: "simple", "complex", or "real_world"
        difficulty: Difficulty level (0.0-1.0)
        width, height: Map dimensions
        resolution: Cell resolution
        
    Returns:
        Dictionary containing scenario data
    """
    scenario = {
        "name": f"{category}_{difficulty:.1f}",
        "category": category,
        "difficulty": difficulty,
        "width": width,
        "height": height,
        "resolution": resolution,
        "obstacles": [],
        "start": [],
        "goal": [],
        "environmental_factors": generate_environmental_factors(difficulty),
        "shipping_lanes": [],
        "description": f"{category.title()} scenario with difficulty {difficulty:.1f}"
    }
    
    # Generate obstacles based on category and difficulty
    if category == "simple":
        # Simple scenarios have basic obstacle patterns
        
        # Add random obstacles
        num_obstacles = int(30 * difficulty)
        scenario["obstacles"].extend(
            generate_random_obstacles(width, height, num_obstacles, min_distance=20.0)
        )
        
        # Maybe add a wall obstacle
        if difficulty > 0.3:
            wall_x1 = np.random.uniform(-width/4, width/4)
            wall_y1 = np.random.uniform(-height/4, height/4)
            wall_x2 = wall_x1 + np.random.uniform(-width/4, width/4)
            wall_y2 = wall_y1 + np.random.uniform(-height/4, height/4)
            
            scenario["obstacles"].extend(
                generate_wall_obstacles(wall_x1, wall_y1, wall_x2, wall_y2, 10.0)
            )
        
        # Force field: simple uniform or random
        force_type = "uniform" if np.random.random() < 0.7 else "random"
        scenario["force_field"] = generate_force_field(width, height, resolution, force_type)
        
        # Simple shipping lanes
        num_lanes = 1 if difficulty > 0.5 else 0
        scenario["shipping_lanes"] = generate_shipping_lanes(width, height, num_lanes)
        
    elif category == "complex":
        # Complex scenarios have more challenging obstacle arrangements
        
        # Add multiple obstacle clusters
        num_clusters = 2 + int(3 * difficulty)
        for _ in range(num_clusters):
            center_x = np.random.uniform(-width/3, width/3)
            center_y = np.random.uniform(-height/3, height/3)
            size = np.random.uniform(50, 150)
            density = 0.2 + 0.5 * difficulty
            
            scenario["obstacles"].extend(
                generate_obstacle_cluster(center_x, center_y, size, density)
            )
        
        # Add multiple wall obstacles to create maze-like structures
        num_walls = int(4 * difficulty)
        for _ in range(num_walls):
            wall_x1 = np.random.uniform(-width/3, width/3)
            wall_y1 = np.random.uniform(-height/3, height/3)
            length = np.random.uniform(100, 300)
            angle = np.random.uniform(0, 2 * np.pi)
            
            wall_x2 = wall_x1 + length * np.cos(angle)
            wall_y2 = wall_y1 + length * np.sin(angle)
            
            scenario["obstacles"].extend(
                generate_wall_obstacles(wall_x1, wall_y1, wall_x2, wall_y2, 10.0)
            )
        
        # Force field: more complex patterns
        force_type = np.random.choice(["vortex", "gradient", "complex"])
        scenario["force_field"] = generate_force_field(width, height, resolution, force_type)
        
        # More shipping lanes
        num_lanes = 2 + int(difficulty * 2)
        scenario["shipping_lanes"] = generate_shipping_lanes(width, height, num_lanes)
        
    else:  # real_world
        # Real-world scenarios mimic actual maritime environments
        
        # Create coastline-like obstacles
        coastline_points = []
        
        # Generate a coastline along one edge
        edge = np.random.randint(0, 4)  # 0: left, 1: top, 2: right, 3: bottom
        
        if edge == 0:  # Left edge
            x_base = -width/2
            for y in np.linspace(-height/2, height/2, 50):
                deviation = np.random.uniform(50, 150) * (0.5 + 0.5 * difficulty)
                coastline_points.append((x_base + deviation, y))
        elif edge == 1:  # Top edge
            y_base = height/2
            for x in np.linspace(-width/2, width/2, 50):
                deviation = np.random.uniform(50, 150) * (0.5 + 0.5 * difficulty)
                coastline_points.append((x, y_base - deviation))
        elif edge == 2:  # Right edge
            x_base = width/2
            for y in np.linspace(-height/2, height/2, 50):
                deviation = np.random.uniform(50, 150) * (0.5 + 0.5 * difficulty)
                coastline_points.append((x_base - deviation, y))
        else:  # Bottom edge
            y_base = -height/2
            for x in np.linspace(-width/2, width/2, 50):
                deviation = np.random.uniform(50, 150) * (0.5 + 0.5 * difficulty)
                coastline_points.append((x, y_base + deviation))
        
        # Add "land" obstacles along coastline
        for x, y in coastline_points:
            # Add multiple obstacles to thicken coastline
            for _ in range(5):
                offset_x = np.random.uniform(-10, 10)
                offset_y = np.random.uniform(-10, 10)
                scenario["obstacles"].append((x + offset_x, y + offset_y))
        
        # Add island-like obstacle clusters
        num_islands = int(3 * difficulty)
        for _ in range(num_islands):
            center_x = np.random.uniform(-width/3, width/3)
            center_y = np.random.uniform(-height/3, height/3)
            size = np.random.uniform(30, 100)
            density = 0.3 + 0.6 * difficulty
            
            scenario["obstacles"].extend(
                generate_obstacle_cluster(center_x, center_y, size, density)
            )
        
        # Add navigation hazards (small obstacle clusters)
        num_hazards = int(10 * difficulty)
        for _ in range(num_hazards):
            center_x = np.random.uniform(-width/2.2, width/2.2)
            center_y = np.random.uniform(-height/2.2, height/2.2)
            size = np.random.uniform(5, 20)
            density = 0.3
            
            scenario["obstacles"].extend(
                generate_obstacle_cluster(center_x, center_y, size, density)
            )
        
        # Force field: ocean current patterns
        force_type = np.random.choice(["gradient", "complex"])
        scenario["force_field"] = generate_force_field(width, height, resolution, force_type)
        
        # Shipping lanes following coastal routes
        num_lanes = 3 + int(difficulty * 3)
        scenario["shipping_lanes"] = generate_shipping_lanes(width, height, num_lanes)
        
        # More detailed description
        scenario["description"] = f"Realistic maritime environment with {'high' if difficulty > 0.7 else 'moderate' if difficulty > 0.4 else 'low'} difficulty"
    
    # Generate start and goal positions that are not on obstacles
    valid_start_goal = False
    max_attempts = 100
    attempt = 0
    
    min_separation = width * 0.4  # Min distance between start and goal
    
    while not valid_start_goal and attempt < max_attempts:
        # Generate candidates
        start_x = np.random.uniform(-width/2.5, width/2.5)
        start_y = np.random.uniform(-height/2.5, height/2.5)
        
        # Generate goal with minimum separation
        while True:
            goal_x = np.random.uniform(-width/2.5, width/2.5)
            goal_y = np.random.uniform(-height/2.5, height/2.5)
            
            dist = np.sqrt((goal_x - start_x)**2 + (goal_y - start_y)**2)
            if dist >= min_separation:
                break
        
        # Check if start and goal are clear of obstacles
        start_clear = True
        goal_clear = True
        clearance = 20.0  # Minimum distance to obstacles
        
        for ox, oy in scenario["obstacles"]:
            start_dist = np.sqrt((start_x - ox)**2 + (start_y - oy)**2)
            goal_dist = np.sqrt((goal_x - ox)**2 + (goal_y - oy)**2)
            
            if start_dist < clearance:
                start_clear = False
            
            if goal_dist < clearance:
                goal_clear = False
            
            if not start_clear and not goal_clear:
                break
        
        valid_start_goal = start_clear and goal_clear
        attempt += 1
    
    if valid_start_goal:
        scenario["start"] = [start_x, start_y]
        scenario["goal"] = [goal_x, goal_y]
    else:
        # Fallback if we couldn't find valid positions
        scenario["start"] = [-width/3, -height/3]
        scenario["goal"] = [width/3, height/3]
        
        # Clear obstacles near start and goal
        clearance = 20.0
        scenario["obstacles"] = [
            (ox, oy) for ox, oy in scenario["obstacles"]
            if (np.sqrt((scenario["start"][0] - ox)**2 + (scenario["start"][1] - oy)**2) >= clearance and
                np.sqrt((scenario["goal"][0] - ox)**2 + (scenario["goal"][1] - oy)**2) >= clearance)
        ]
    
    return scenario

def visualize_scenario(scenario, output_path):
    """
    Visualize the scenario and save to a file
    
    Args:
        scenario: Scenario dictionary
        output_path: Path to save visualization
    """
    plt.figure(figsize=(10, 10))
    
    # Set plot limits
    width = scenario["width"]
    height = scenario["height"]
    plt.xlim(-width/2, width/2)
    plt.ylim(-height/2, height/2)
    
    # Plot obstacles
    obstacles_x = [o[0] for o in scenario["obstacles"]]
    obstacles_y = [o[1] for o in scenario["obstacles"]]
    plt.scatter(obstacles_x, obstacles_y, c='gray', s=10, label='Obstacles')
    
    # Plot start and goal
    plt.scatter(scenario["start"][0], scenario["start"][1], c='green', s=100, marker='o', label='Start')
    plt.scatter(scenario["goal"][0], scenario["goal"][1], c='red', s=100, marker='x', label='Goal')
    
    # Plot shipping lanes
    for lane in scenario["shipping_lanes"]:
        lane_x = [p[0] for p in lane]
        lane_y = [p[1] for p in lane]
        plt.plot(lane_x, lane_y, 'b--', alpha=0.6, linewidth=1.5)
    
    # Plot force field (sample points)
    if "force_field" in scenario and scenario["force_field"]:
        # Sample a subset of force field points
        force_field = scenario["force_field"]
        sample_size = min(100, len(force_field))
        keys = random.sample(list(force_field.keys()), sample_size)
        
        for key in keys:
            x, y = map(float, key.split('_'))
            magnitude = force_field[key]["magnitude"]
            direction = force_field[key]["direction"]
            
            # Create arrow
            dx = magnitude * np.cos(direction) * 20  # Scale arrow
            dy = magnitude * np.sin(direction) * 20
            plt.arrow(x, y, dx, dy, head_width=5, head_length=7, fc='blue', ec='blue', alpha=0.3)
    
    # Add title and labels
    plt.title(f"{scenario['name']}: {scenario['description']}")
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    # Save figure
    plt.tight_layout()
    plt.savefig(output_path)
    plt.close()

def save_scenario(scenario, output_dir):
    """
    Save scenario data to a JSON file
    
    Args:
        scenario: Scenario dictionary
        output_dir: Directory to save scenario
    """
    # Create filename
    filename = f"{scenario['name']}.scenario"
    filepath = os.path.join(output_dir, filename)
    
    # Save as JSON
    with open(filepath, 'w') as f:
        json.dump(scenario, f, indent=2)
    
    # Create visualization
    vis_dir = "test_environments/visualizations"
    vis_path = os.path.join(vis_dir, f"{scenario['name']}.png")
    visualize_scenario(scenario, vis_path)
    
    print(f"Saved scenario: {filename}")

def main():
    parser = argparse.ArgumentParser(description='Generate test scenarios for ASV planning')
    parser.add_argument('--simple', type=int, default=3, help='Number of simple scenarios')
    parser.add_argument('--complex', type=int, default=3, help='Number of complex scenarios')
    parser.add_argument('--real_world', type=int, default=3, help='Number of real-world scenarios')
    parser.add_argument('--seed', type=int, default=None, help='Random seed for reproducibility')
    
    args = parser.parse_args()
    
    # Set random seed if provided
    if args.seed is not None:
        np.random.seed(args.seed)
        random.seed(args.seed)
    
    # Create directory structure
    create_directory_structure()
    
    # Generate scenarios for each category
    for category, count in [
        ("simple", args.simple),
        ("complex", args.complex),
        ("real_world", args.real_world)
    ]:
        output_dir = f"test_environments/{category}"
        
        for i in range(count):
            # Calculate difficulty (spread between 0.1 and 0.9)
            difficulty = 0.1 + 0.8 * i / (count - 1) if count > 1 else 0.5
            
            # Generate scenario
            scenario = generate_scenario(category, difficulty)
            
            # Save scenario
            save_scenario(scenario, output_dir)
    
    print(f"\nGenerated {args.simple} simple, {args.complex} complex, and {args.real_world} real-world scenarios")
    print("Visualizations saved in test_environments/visualizations")

if __name__ == "__main__":
    main() 