/*
 * linmath_test.h
 *
 *  Created on: Apr 9, 2017
 *      Author: dizcza
 */

#ifndef LINMATH_H_LINMATH_TEST_H_
#define LINMATH_H_LINMATH_TEST_H_

#include "linmath.h"
#include "linmath_additional.h"

#ifndef linmath_assert
#include <assert.h>
#define linmath_assert  assert
#endif /* linmath_assert */

#define LINMATH_EPS                       (0.0001f)


#define LINMATH_TEST_M(n) \
static inline int vec##n##_allclose(vec##n const a, vec##n const b) { \
	int i, equal = 1U; \
	for(i = 0; i < n; ++i) \
		equal &= (fabsf(a[i] - b[i]) < LINMATH_EPS); \
	return equal; \
} \
static inline int mat##n##x##n##_allclose(mat##n##x##n const M, mat##n##x##n const N) { \
	int i, equal = 1U; \
	for (i = 0; i < n; ++i) \
		equal &= vec##n##_allclose(M[i], N[i]); \
	return equal; \
}

LINMATH_TEST_M(3);
LINMATH_TEST_M(4);

static void Linmath_TestQuatMat4() {
	quat q;
	vec3 axis = { 0, 1, 0 };
	float degrees = 15;
	quat_rotate(q, LINMATH_DEGREES_TO_RADS(degrees), axis);
	mat4x4 m_ident, m_rotated, m;
	mat4x4_identity(m_ident);
	mat4x4o_mul_quat(m_rotated, m_ident, q);
	quat_rotate(q, LINMATH_DEGREES_TO_RADS(-degrees), axis);
	mat4x4o_mul_quat(m, m_rotated, q);
	int eq = mat4x4_allclose(m_ident, m);
	linmath_assert(eq);
}

static void Linmath_TestQuatVec3() {
	quat q;
	vec3 axis = { 1, 1, -1 };
	float degrees = 139.27f;
	quat_rotate(q, LINMATH_DEGREES_TO_RADS(degrees), axis);
	const vec3 v_ref = { 23.12f, 0.89f, -6.23f };
	vec3 v;
	vec3_dup(v, v_ref);

	vec3 v_clone;
	vec3_dup(v_clone, v);

	quat_mul_vec3(v, q, v_clone);
	quat_rotate(q, LINMATH_DEGREES_TO_RADS(-degrees), axis);

	vec3_dup(v_clone, v);
	quat_mul_vec3(v, q, v_clone);
	int eq = vec3_allclose(v_ref, v);
	linmath_assert(eq);
}

static void Linmath_RunTests() {
	Linmath_TestQuatVec3();
}

#endif /* LINMATH_H_LINMATH_TEST_H_ */
