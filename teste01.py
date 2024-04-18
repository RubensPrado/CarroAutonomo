import cv2

def main():
    # Configuração da câmera
    cap = cv2.VideoCapture(0)  # Substitua 0 pelo índice correto da câmera, se necessário

    if not cap.isOpened():
        print("Erro ao abrir a câmera.")
        return

    try:
        while True:
            # Captura um frame
            ret, frame = cap.read()
            if not ret:
                print("Falha ao capturar imagem da câmera.")
                break

            # Exibe o frame
            cv2.imshow('Camera Feed', frame)

            # Fecha a janela com a tecla 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        # Libera a câmera e destrói todas as janelas abertas
        cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()