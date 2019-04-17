#include "util.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>


void matrixFromUpDir(Mtxfp dst, v3f *upDir, v3f *pos, s16 yaw) {
  v3f faceDir;
  setV3f(&faceDir, sins(yaw), 0.0f, coss(yaw));
  
  normalizeV3f(upDir);

  v3f leftDir;
  crossProduct(&leftDir, upDir, &faceDir);
  normalizeV3f(&leftDir);

  v3f forwardDir;
  crossProduct(&forwardDir, &leftDir, upDir);
  normalizeV3f(&forwardDir);
  
  dst[0][0] = leftDir.x;
  dst[0][1] = leftDir.y;
  dst[0][2] = leftDir.z;
  dst[3][0] = pos->x;
  
  dst[1][0] = upDir->x;
  dst[1][1] = upDir->y;
  dst[1][2] = upDir->z;
  dst[3][1] = pos->y;
  
  dst[2][0] = forwardDir.x;
  dst[2][1] = forwardDir.y;
  dst[2][2] = forwardDir.z;
  dst[3][2] = pos->z;

  dst[0][3] = 0.0f;
  dst[1][3] = 0.0f;
  dst[2][3] = 0.0f;
  dst[3][3] = 1.0f;
}


void buildTiltTransform(Mtxfp m, f32 nx, f32 ny, f32 nz) {
  v3f n = {nx, ny, nz};
  v3f pos = {0, 0, 0};
  matrixFromUpDir(m, &n, &pos, 0);
}


f32 incTowardSymFS(f32 target, f32 x, f32 delta) {
  if (x <= target) {
    if (target - x < delta)
      return target;
    else
      return x + delta;
  }
  else {
    if (target - x > delta)
      return target;
    else
      return x - delta;
  }
}


f32 phi(v3f v) {
  return atan2(-v.x, v.y);
}

f32 theta(v3f v) {
  return asin(v.z);
}


f32 valueForNormal(v3f offset, v3f normal) {
  Mtxf transform;
  buildTiltTransform(&transform, normal.x, normal.y, normal.z);

  v3f transformedOffset;
  matrixVecMult(&transform, &transformedOffset, &offset);

  v3f targetNormal = { offset.x, 500.0f, offset.z };

  f32 mag = sqrtf(
    targetNormal.x * targetNormal.x +
    targetNormal.y * targetNormal.y +
    targetNormal.z * targetNormal.z);

  if (mag != 0.0f) {
    mag = (f32) (1.0 / mag);
    targetNormal.x *= mag;
    targetNormal.y *= mag;
    targetNormal.z *= mag;
  }
  else {
    targetNormal.x = 0.0f;
    targetNormal.y = 1.0f;
    targetNormal.z = 0.0f;
  }
  // printf("target normal: %f %f %f (%f %f)\n", targetNormal.x, targetNormal.y, targetNormal.z, phi(targetNormal), theta(targetNormal));

  normal.x = incTowardSymFS(targetNormal.x, normal.x, 0.01f);
  normal.y = incTowardSymFS(targetNormal.y, normal.y, 0.01f);
  normal.z = incTowardSymFS(targetNormal.z, normal.z, 0.01f);
  // printf("normal': %f %f %f (%f %f)\n", normal.x, normal.y, normal.z, phi(normal), theta(normal));

  buildTiltTransform(&transform, normal.x, normal.y, normal.z);

  v3f newTransformedOffset;
  matrixVecMult(&transform, &newTransformedOffset, &offset);
  
  // printf("dy: %f\n", newTransformedOffset.y - transformedOffset.y);
  return newTransformedOffset.y - transformedOffset.y;
}

bool checkNormal(v3f offset, v3f normal) {
  return valueForNormal(offset, normal) > 0;
}


v3f findPosForTargetNormal(v3f n) {
  f32 mag = 500.0f / n.y;
  n.x *= mag;
  n.y *= mag;
  n.z *= mag;
  return n;
}


// f32 computeQpuSpeed(s16 angle) {
//   v3f pos = { coss(angle) }
// }


s32 signum(f32 x) {
  if (x > 0.00001) return 1;
  if (x < -0.00001) return -1;
  return 0;
}


f32 findSpeedForAngle(s16 a) {
  for (f32 sp = 65536.0f; sp < 400000000; sp += 1) {
    f32 x = sp * sins(a);
    f32 z = sp * coss(a);
    s16 x0 = (s32) x;
    s16 z0 = (s32) z;
    if (x0 < -100 || x0 > 100) continue;
    if (z0 < -100 || z0 > 100) continue;
    return sp;
  }
}


int main(void) {
  // f32 puSpeeds[0x1000];
  // for (s32 a = 0; a < 0x10000; a += 0x100) {
  //   puSpeeds[a / 0x10] = findSpeedForAngle(a);
  // }
  // printf("Computed speeds\n");
  // return 0;

  // v3f p = findPosForTargetNormal((v3f) {-0.224980, 0.801901, -0.553479});
  // printf("%f %f\n", p.x - 2866, p.z - 715);
  // return 0;

  // // f32 x = -66987.867188;
  // // f32 z = -253440.531250;
  // // s16 a = atan2xy(z, x);
  // // printf("%d\n", a);
  // // return 0;

  // f32 d = 0.0005f;
  // f32 d2 = 0.01f;
  // f32 minNy = 0.01f;
  // f32 mag = 4 * 65536.0f;

  // // v3f n = {-0.2111258, 0.8025488, 0.5579796};
  // // // v3f pos = {0, 0, -366.78 + -262144};

  // // // printf("%d\n", checkNormal(pos, n));
  // // // return 0;

  // f32 maxNy = 0.0f;

  // for (f32 nx = -1.0f; nx <= 1.0f + d; nx += d) {
  //   // for (f32 ny = minNy; ny <= 1.0f + d; ny += d) {
  //     for (f32 nz = -1.0f; nz <= 1.0f + d; nz += d) {
  //       f32 ny = sqrtf(1 - nx*nx - nz*nz);
  //       if (!(ny >= 0.80)) continue;
  //       // f32 len = sqrtf(nx*nx + ny*ny + nz*nz);
  //       // if (len > 1 || len < 0.99) continue;
  //       v3f n = {nx, ny, nz};

  //       // for (f32 a = 0.0f; a <= 6.283 + d2; a += d2) {
  //       // for (s32 pux = -10; pux <= 10; pux += 1) {
  //       //   for (s32 puz = -10; puz <= 10; puz += 1) {
  //       for (s32 a = 0; a < 0x10000; a += 0x2000) {
  //         f32 puSpeed = 65536.0f; //puSpeeds[a / 0x10];

  //           // s16 a = atan2xy(puz, pux);
  //           // f32 pumag = 4 * sqrtf(pux*pux + puz*puz) * 65536.0f;
  //           // v3f pos = {pumag * sins(a), 0, pumag * coss(a)};
  //         v3f pos = {4*puSpeed * signum(sins(a)), 0, 4*puSpeed * signum(coss(a))};

  //           if (checkNormal(pos, n) && n.y > maxNy) {
  //             maxNy = n.y;
  //             printf("Got one:\n");
  //             printf("  n = (%f, %f, %f)\n", n.x, n.y, n.z);
  //             printf("  a = %d\n", a);
  //             printf("  p = (%f, %f, %f)\n", pos.x, pos.y, pos.z);
  //           }
  //         }
  //       }
  //     // }
  //   // }
  // }

  // v3f pos = {0, 0, 262144};
  // v3f pos = {-66987, 0.000000, -253440};
  f32 minny = 0.71;

  // v3f n = {-0.254980, 0.801901, -0.553479};
  v3f n = {0, 1, 0};
  f32 vmax = -10000000;

  // while (1) {
  //   v3f newn = {n.x, n.y, n.z};
  //   f32 amount = (((f32)rand() / RAND_MAX) * 2 - 1) * 0.1f;
  //   if (rand() % 2 == 0) {
  //     newn.x += amount;
  //   } else {
  //     newn.z += amount;
  //   }
  //   normalizeV3f(&newn);
  //   if (newn.y < minny) {
  //     continue;
  //   }

  //   f32 v = valueForNormal(pos, newn);
  //   if (v < vmax) {
  //     continue;
  //   }

  //   n = newn;
  //   vmax = v;
  //   printf("N = %f %f %f, v = %f\n", n.x, n.y, n.z, vmax);
  // }

  for (f32 b = 0; b < 2 * 3.141592653f; b += 0.001) {
    f32 r = 100000.0f;
    v3f pos = {r * sin(b), 0, r * cos(b)};

    for (f32 a = 0; a < 2 * 3.141592653f; a += 0.001) {
      f32 xz = sqrtf(1 - minny*minny);
      v3f newn = {xz * sin(a), minny, xz * cos(a)};
      normalizeV3f(&newn);

      f32 v = valueForNormal(pos, newn);
      if (v < vmax) {
        continue;
      }

      n = newn;
      vmax = v;
      printf("N = %f %f %f, v = %f\n", n.x, n.y, n.z, vmax);
    }
  }

  // normalizeV3f(&n);
  // printf("N = %f %f %f\n", n.x, n.y, n.z);

  // printf("%d\n", checkNormal(pos, n));

  return 0;
}


// Got one:
//   n = (-0.270001, 0.849999, 0.449999)
//   p = (-66987.867188, 0.000000, -253440.531250)


// Got one:
//   n = (0.219999, 0.854811, 0.469999)
//   a = 30483
//   p = (525916.500000, 0.000000, -2358933.500000)


// n = (-0.210980, 0.802213, 0.558517)
// a = 32768
// p = (0.000000, 0.000000, -262144.000000)


// n = (-0.224980, 0.801901, -0.553479)
// a = 0
// p = (0.000000, 0.000000, 262144.000000)
