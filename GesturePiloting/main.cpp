#include "NiTE.h"
#include "util.h"
#include <windows.h>

#define EXPORT extern "C" __declspec(dllexport)
#define FACTOR_CONFIANZA .5
#define MIN_DEG_HOMBRO_THROTTLE 40.
#define MIN_DEG_YAW_DERECHA 35
#define MIN_DEG_YAW_IZQUIERDA 45
#define MIN_DEG_PITCH_ABAJO 25
#define MIN_DEG_PITCH_ARRIBA 40
#define VARIACION_THROTTLE_PERMITIDA 0.04
#define DISTANCIA_MAXIMA_DESARME 100

float ultimo_throttle = 1000;
nite::UserId idPrimerUsuario;

float throttle = 1000;
float yaw = 0;
float pitch = 0;
float roll = 0;

bool finish = false;

EXPORT float GetThrottle() {
	return throttle;
}

EXPORT float GetYaw() {
	return yaw;
}

EXPORT float GetPitch() {
	return pitch;
}

EXPORT float GetRoll() {
	return roll;
}

EXPORT void Stop() {
	finish = true;
}

bool is_throttle_valido(nite::SkeletonJoint hombro_izq, nite::SkeletonJoint codo_izq) {
	// solo con x e y
	nite::Point3f hombro = nite::Point3f(hombro_izq.getPosition().x, hombro_izq.getPosition().y, 0);
	nite::Point3f codo = nite::Point3f(codo_izq.getPosition().x, codo_izq.getPosition().y, 0);

	nite::Point3f aux = nite::Point3f();
	aux.x = hombro.x;
	aux.y = codo.y;
	aux.z = 0;

	float a = distancia_euclidea(codo, hombro);
	float c = distancia_euclidea(aux, codo);
	float sinalpha = c / a;

	float alpha = rad_to_deg(asin(sinalpha));

	return alpha > MIN_DEG_HOMBRO_THROTTLE;
}

float calculate_throttle(nite::SkeletonJoint mano_izq, nite::SkeletonJoint codo_izq) {
	// solo con z e y
	int signo = 1;
	nite::Point3f mano = nite::Point3f(0, mano_izq.getPosition().y, mano_izq.getPosition().z);
	nite::Point3f codo = nite::Point3f(0, codo_izq.getPosition().y, codo_izq.getPosition().z);

	nite::Point3f aux = nite::Point3f();
	aux.x = 0;
	aux.y = codo.y;
	aux.z = mano.z;

	if (mano.y < codo.y) {
		signo = -1;
	}

	float a = distancia_euclidea(codo, mano);
	float c = distancia_euclidea(aux, mano);
	float sinalpha = c / a;
	return signo*rad_to_deg(asin(sinalpha));
}

float calculate_pitch(nite::SkeletonJoint mano_dcha, nite::SkeletonJoint codo_dcho) {
	bool arriba = false;
	nite::Point3f mano = nite::Point3f(0, mano_dcha.getPosition().y, mano_dcha.getPosition().z);
	nite::Point3f codo = nite::Point3f(0, codo_dcho.getPosition().y, codo_dcho.getPosition().z);

	nite::Point3f aux = nite::Point3f();
	aux.x = 0;
	aux.y = codo.y;
	aux.z = mano.z;

	arriba = mano.y > codo.y;

	float a = distancia_euclidea(codo, mano);
	float c = distancia_euclidea(aux, mano);
	float sinalpha = c / a;
	float deg = rad_to_deg(asin(sinalpha));

	if (arriba) {
		if (deg > MIN_DEG_PITCH_ARRIBA) {
			return 1;
		}
	}
	else {
		if (deg > MIN_DEG_PITCH_ABAJO) {
			return -1;
		}
	}

	return 0;

}

float calculate_roll(nite::SkeletonJoint mano_der, nite::SkeletonJoint codo_der) {
	nite::Point3f mano = nite::Point3f(mano_der.getPosition().x, 0, mano_der.getPosition().z);
	nite::Point3f codo = nite::Point3f(codo_der.getPosition().x, 0, codo_der.getPosition().z);

	nite::Point3f aux = nite::Point3f();
	aux.x = mano.x;
	aux.y = 0;
	aux.z = codo.z;

	float a = distancia_euclidea(mano, aux);
	float c = distancia_euclidea(mano, codo);
	float sinalpha = a / c;
	float deg = rad_to_deg(asin(sinalpha));

	// izquierda (abrir brazo)
	if (mano.x < codo.x) {
		if (deg < MIN_DEG_YAW_IZQUIERDA) {
			return -1;
		}
	}
	// derecha (flexionar brazo)
	else {
		if (deg < MIN_DEG_YAW_DERECHA) {
			return 1;
		}
	}

	// brazo recto
	return 0;
}

bool desarmar(nite::SkeletonJoint mano_izq, nite::SkeletonJoint mano_der) {
	nite::Point3f izq = nite::Point3f(mano_izq.getPosition().x, mano_izq.getPosition().y, mano_izq.getPosition().z);
	nite::Point3f der = nite::Point3f(mano_der.getPosition().x, mano_der.getPosition().y, mano_der.getPosition().z);

	float dist = distancia_euclidea(izq, der);
	//printf("\nDIST: %f", dist);
	return dist<DISTANCIA_MAXIMA_DESARME;
}

float calculate_yaw(nite::SkeletonJoint mano_izq, nite::SkeletonJoint codo_izq) {
	nite::Point3f mano = nite::Point3f(mano_izq.getPosition().x, 0, mano_izq.getPosition().z);
	nite::Point3f codo = nite::Point3f(codo_izq.getPosition().x, 0, codo_izq.getPosition().z);

	nite::Point3f aux = nite::Point3f();
	aux.x = mano.x;
	aux.y = 0;
	aux.z = codo.z;

	float a = distancia_euclidea(mano, aux);
	float c = distancia_euclidea(mano, codo);
	float sinalpha = a / c;
	float deg = rad_to_deg(asin(sinalpha));

	// izquierda (abrir brazo)
	if (mano.x < codo.x) {
		if (deg < MIN_DEG_YAW_IZQUIERDA) {
			return -1;
		}
	}
	// derecha (flexionar brazo)
	else {
		if (deg < MIN_DEG_YAW_DERECHA) {
			return 1;
		}
	}

	// brazo recto
	return 0;
}

void cerrar() {
	nite::NiTE::shutdown();
}


EXPORT int Initialize()
{
	// Inicializacion de la Kinect
	printf("Iniciando tracking...");
	nite::UserTracker tracker;
	nite::Status estado;
	bool usuarioGuardado = false;
	finish = false;

	nite::NiTE::initialize();

	estado = tracker.create();
	if (estado != nite::STATUS_OK)
	{
		printf("Error.\nNo se ha podido arrancar el programa. Comprueba las carpetas OpenNi2 y NiTE2 que estén copiadas en $Output y proyecto");
		return -1;
	}
	printf(" ok!");
	// Fin inicializacion de Kinect

	printf("\n\nTratando de detectar al usuario...");

	nite::UserTrackerFrameRef frame;
	while (!wasKeyboardHit()&& !finish)
	{
		estado = tracker.readFrame(&frame);
		if (estado != nite::STATUS_OK)
		{
			printf("Error en el frame\n");
			continue;
		}

		const nite::Array<nite::UserData>& users = frame.getUsers();

		// si el usuario sale, precipitar el UAV.
		if (users.getSize() == 0) {
			throttle = 1000;
		}

		for (int i = 0; i < users.getSize(); ++i)
		{

			const nite::UserData& user = users[i];

			if (user.isNew())
			{
				tracker.startSkeletonTracking(user.getId());
			}
			else if (user.getSkeleton().getState() == nite::SKELETON_TRACKED)
			{

				// throttle
				const nite::SkeletonJoint& mano_izq = user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND);
				const nite::SkeletonJoint& codo_izq = user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW);
				const nite::SkeletonJoint& hombro_izq = user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER);


				// yaw
				const nite::SkeletonJoint&  mano_der = user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND);
				const nite::SkeletonJoint&  codo_der = user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW);

				// roll
				const nite::SkeletonJoint& cabeza = user.getSkeleton().getJoint(nite::JOINT_HEAD);
				const nite::SkeletonJoint& cadera_izq = user.getSkeleton().getJoint(nite::JOINT_LEFT_HIP);
				const nite::SkeletonJoint& cadera_der = user.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP);

				bool orden_desarme = false;

				if (mano_der.getPositionConfidence() > .5 && mano_izq.getPositionConfidence() > .5) {
					orden_desarme = desarmar(mano_izq, mano_der);
				}

				if (mano_der.getPositionConfidence() > .5 && codo_der.getPositionConfidence() > .5) {
					pitch = calculate_pitch(mano_der, codo_der);
				}

				if (mano_der.getPositionConfidence() > .5 && codo_der.getPositionConfidence() > .5 /*&& throttle_valido(hombro_izq, codo_izq)*/) {
					roll = calculate_roll(mano_der, codo_der);
				}

				if (mano_izq.getPositionConfidence() > .5 && codo_izq.getPositionConfidence() > .5 && is_throttle_valido(hombro_izq, codo_izq)) {
					yaw = calculate_yaw(mano_izq, codo_izq);
				}

				if (mano_izq.getPositionConfidence() > .5 && codo_izq.getPositionConfidence() > .5 /*&& throttle_valido(hombro_izq, codo_izq)*/) {
					throttle = ang_to_throttle(calculate_throttle(mano_izq, codo_izq));
					float resta = fabs(throttle - ultimo_throttle);
					if (resta > ultimo_throttle*VARIACION_THROTTLE_PERMITIDA) {
						ultimo_throttle = throttle;
					}
					else {
						throttle = ultimo_throttle;
					}

				}

				if (orden_desarme) {
					throttle = 0;
					finish = true;
					break;
				}

				printf("%f %f %f %f\n", throttle, yaw, pitch, roll);
			}
		}

	}


	nite::NiTE::shutdown();

}
