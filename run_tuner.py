import sys

try:
    import numpy as np
    from sklearn.ensemble import RandomForestRegressor
    import requests
except ImportError:
    print("Missing required packages")
    print("\nPlease run:")
    print("pip install numpy scikit-learn requests")
    sys.exit(1)

import time
import json
from collections import deque


class SimpleAutoTuner:
    def __init__(self):
        self.esp32_ip = "192.168.4.1"
        self.base_url = f"http://{self.esp32_ip}"
        
    def check_connection(self):
        """Check if we can reach the ESP32"""
        print("Checking connection to ESP32...")
        try:
            response = requests.get(f"{self.base_url}/data", timeout=2)
            if response.status_code == 200:
                print("Connected to ESP32!")
                data = response.json()
                print(f"   Current distance: {data['distance']:.2f} cm")
                print(f"   Current servo positions: {data['servo1']:.1f}°, {data['servo2']:.1f}°")
                return True
        except requests.exceptions.ConnectionError:
            print("Cannot connect to ESP32!")
            print("\nTroubleshooting:")
            print("   1. Is ESP32 powered on?")
            print("   2. Did you connect to WiFi 'ESP32-siva_the_goat'?")
            print("   3. Can you open http://192.168.4.1 in your browser?")
            return False
        except Exception as e:
            print(f"Error: {e}")
            return False
    
    def test_params(self, kp, ki, kd, test_duration=8):
        # Set parameters
        params = {"kP": kp, "kI": ki, "kD": kd, "target": 15.0}
        requests.post(f"{self.base_url}/update_pid", json=params)
        
        # Wait for stabilization
        time.sleep(2)
        
        # Collect data
        errors = []
        start = time.time()
        
        while time.time() - start < test_duration:
            try:
                response = requests.get(f"{self.base_url}/data", timeout=1)
                data = response.json()
                errors.append(abs(data['tiltError']))
                time.sleep(0.1)
            except:
                continue
        
        if not errors:
            return None
        
        mean_error = np.mean(errors)
        std_error = np.std(errors)
        score = mean_error * 2 + std_error
        
        return {
            'score': score,
            'mean_error': mean_error,
            'std_error': std_error
        }
    
    def run_quick_tune(self):
        """Quick tuning with 10 tests"""
        print("\n" + "="*60)
        print("QUICK PID AUTO-TUNING")
        print("="*60)
        print("This will test 10 parameter combinations (~2 minutes)")
        input("Press ENTER to start...")
        
        # Test candidates
        candidates = [
            {'kP': 0.32, 'kI': 0.02, 'kD': 0.02},
            {'kP': 0.52, 'kI': 0.02, 'kD': 0.02},
            {'kP': 0.82, 'kI': 0.02, 'kD': 0.02},
            {'kP': 0.52, 'kI': 0.07, 'kD': 0.02},
            {'kP': 0.52, 'kI': 0.12, 'kD': 0.02},  #kP = 0.52, kI = 0.02, kD = 0.02
            {'kP': 0.52, 'kI': 0.02, 'kD': 0.22},
            {'kP': 0.52, 'kI': 0.02, 'kD': 0.52},
            {'kP': 0.72, 'kI': 0.07, 'kD': 0.32},
            {'kP': 0.40, 'kI': 0.08, 'kD': 0.2},
            {'kP': 0.62, 'kI': 0.05, 'kD': 0.42},
        ]
        
        best_score = float('inf')
        best_params = None
        results = []
        
        for i, params in enumerate(candidates):
            print(f"\n Test {i+1}/10: kP={params['kP']:.2f}, kI={params['kI']:.3f}, kD={params['kD']:.2f}")
            
            try:
                result = self.test_params(params['kP'], params['kI'], params['kD'])
                
                if result:
                    print(f"   Score: {result['score']:.2f} | Error: {result['mean_error']:.2f}cm ± {result['std_error']:.2f}cm")
                    
                    if result['score'] < best_score:
                        best_score = result['score']
                        best_params = params
                        print(f"    New best!")
                    
                    results.append({'params': params, 'result': result})
                else:
                    print(f"Test failed")
                    
            except Exception as e:
                print(f"Error: {e}")
                continue
        
        # Show results
        print("\n" + "="*60)
        print("TUNING COMPLETE!")
        print("="*60)
        
        if best_params:
            print(f"\nBest Parameters:")
            print(f"   kP = {best_params['kP']:.3f}")
            print(f"   kI = {best_params['kI']:.4f}")
            print(f"   kD = {best_params['kD']:.3f}")
            print(f"\nPerformance:")
            
            best_result = next(r['result'] for r in results if r['params'] == best_params)
            print(f"   Mean Error: {best_result['mean_error']:.2f} cm")
            print(f"   Std Dev: {best_result['std_error']:.2f} cm")
            print(f"   Score: {best_result['score']:.2f}")
            
            # Apply best parameters
            print("\nApplying best parameters to ESP32...")
            requests.post(f"{self.base_url}/update_pid", json={
                "kP": best_params['kP'],
                "kI": best_params['kI'],
                "kD": best_params['kD'],
                "target": 15.0
            })
            print("Done! Check your dashboard to see the improvement.")
            
            # Save results
            with open('tuning_results.json', 'w') as f:
                json.dump({
                    'best_params': best_params,
                    'best_result': best_result,
                    'all_results': results
                }, f, indent=2)
            print("\nResults saved to tuning_results.json")
            
        else:
            print("No successful tests. Check your setup and try again.")
        
        print("="*60 + "\n")


def main():
    print("\n" + "="*60)
    print("        ESP32 PID AUTO-TUNER")
    print("="*60 + "\n")
    
    tuner = SimpleAutoTuner()
    
    # Check connection first
    if not tuner.check_connection():
        print("\nSetup required before tuning!")
        sys.exit(1)
    
    print("\nReady to tune!")
    
    # Ask user
    print("\nChoose tuning mode:")
    print("  1) Quick tune (10 tests, ~2 minutes)")
    print("  2) Cancel")
    
    choice = input("\nEnter choice (1 or 2): ").strip()
    
    if choice == "1":
        tuner.run_quick_tune()
    else:
        print("Cancelled.")
if __name__ == "__main__":
    main()
