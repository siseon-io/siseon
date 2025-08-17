pipeline {
  agent any

  environment {
    COMPOSE_IMAGE = 'docker/compose:1.29.2'
    SERVICE_DIR   = "${WORKSPACE}/siseon-backend"
  }

  stages {
    stage('Checkout') {
      steps { checkout scm }
    }

    stage('Build & Deploy') {
      steps {
        sh """
          # Compose down
          docker run --rm \\
            -v /var/run/docker.sock:/var/run/docker.sock \\
            -v "${SERVICE_DIR}":/app \\
            -w /app \\
            ${COMPOSE_IMAGE} down

          # Compose up
          docker run --rm \\
            -v /var/run/docker.sock:/var/run/docker.sock \\
            -v "${SERVICE_DIR}":/app \\
            -w /app \\
            ${COMPOSE_IMAGE} up -d --build
        """
      }
    }
  }

  post {
    success { echo '✅ 배포 완료!' }
    failure { echo '❌ 배포 실패…' }
  }
}