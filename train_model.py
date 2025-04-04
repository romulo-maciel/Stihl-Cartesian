# train_model.py (versão corrigida)
import numpy as np
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
import joblib

def train_polynomial_model():
    data = np.load("calib_data.npy", allow_pickle=True).item()
    
    # Extrai dados e verifica se estão no formato correto
    X_cam = np.array(data["cam"])
    X_cnc = np.array(data["cnc"])
    
    if X_cam.size == 0 or X_cnc.size == 0:
        raise ValueError("Dados de calibração vazios. Execute calibrate_cnc.py primeiro!")
    
    # Garante que X_cam seja 2D (n_amostras, 2)
    if X_cam.ndim == 1:
        X_cam = X_cam.reshape(-1, 2)
    
    poly = PolynomialFeatures(degree=2)
    X_poly = poly.fit_transform(X_cam)  # Agora X_cam é 2D (ex: (4, 2))
    
    model = LinearRegression()
    model.fit(X_poly, X_cnc)

    print("Formato de X_cam:", X_cam.shape)  # Deve ser (n, 2)
    print("Formato de X_cnc:", X_cnc.shape)  # Deve ser (n, 2)
    
    joblib.dump((model, poly), "calib_model.pkl")
    print("Modelo treinado e salvo!")

if __name__ == "__main__":
    train_polynomial_model()