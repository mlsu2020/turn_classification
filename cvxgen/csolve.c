/* Produced by CVXGEN, 2019-07-09 09:48:03 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: csolve.c. */
/* Description: mex-able file for running cvxgen solver. */
#include "mex.h"
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  int i, j;
  mxArray *xm, *cell, *xm_cell;
  double *src;
  double *dest;
  double *dest_cell;
  int valid_vars;
  int steps;
  int this_var_errors;
  int warned_diags;
  int prepare_for_c = 0;
  int extra_solves;
  const char *status_names[] = {"optval", "gap", "steps", "converged"};
  mwSize dims1x1of1[1] = {1};
  mwSize dims[1];
  const char *var_names[] = {"u_0", "u_1", "u_2", "u_3", "u_4", "u_5", "u_6", "u_7", "u_8", "u_9", "u_10", "u_11", "u_12", "x_1", "x_2", "x_3", "x_4", "x_5", "x_6", "x_7", "x_8", "x_9", "x_10", "x_11", "x_12", "x_13", "u", "x"};
  const int num_var_names = 28;
  /* Avoid compiler warnings of unused variables by using a dummy assignment. */
  warned_diags = j = 0;
  extra_solves = 0;
  set_defaults();
  /* Check we got the right number of arguments. */
  if (nrhs == 0)
    mexErrMsgTxt("Not enough arguments: You need to specify at least the parameters.\n");
  if (nrhs > 1) {
    /* Assume that the second argument is the settings. */
    if (mxGetField(prhs[1], 0, "eps") != NULL)
      settings.eps = *mxGetPr(mxGetField(prhs[1], 0, "eps"));
    if (mxGetField(prhs[1], 0, "max_iters") != NULL)
      settings.max_iters = *mxGetPr(mxGetField(prhs[1], 0, "max_iters"));
    if (mxGetField(prhs[1], 0, "refine_steps") != NULL)
      settings.refine_steps = *mxGetPr(mxGetField(prhs[1], 0, "refine_steps"));
    if (mxGetField(prhs[1], 0, "verbose") != NULL)
      settings.verbose = *mxGetPr(mxGetField(prhs[1], 0, "verbose"));
    if (mxGetField(prhs[1], 0, "better_start") != NULL)
      settings.better_start = *mxGetPr(mxGetField(prhs[1], 0, "better_start"));
    if (mxGetField(prhs[1], 0, "verbose_refinement") != NULL)
      settings.verbose_refinement = *mxGetPr(mxGetField(prhs[1], 0,
            "verbose_refinement"));
    if (mxGetField(prhs[1], 0, "debug") != NULL)
      settings.debug = *mxGetPr(mxGetField(prhs[1], 0, "debug"));
    if (mxGetField(prhs[1], 0, "kkt_reg") != NULL)
      settings.kkt_reg = *mxGetPr(mxGetField(prhs[1], 0, "kkt_reg"));
    if (mxGetField(prhs[1], 0, "s_init") != NULL)
      settings.s_init = *mxGetPr(mxGetField(prhs[1], 0, "s_init"));
    if (mxGetField(prhs[1], 0, "z_init") != NULL)
      settings.z_init = *mxGetPr(mxGetField(prhs[1], 0, "z_init"));
    if (mxGetField(prhs[1], 0, "resid_tol") != NULL)
      settings.resid_tol = *mxGetPr(mxGetField(prhs[1], 0, "resid_tol"));
    if (mxGetField(prhs[1], 0, "extra_solves") != NULL)
      extra_solves = *mxGetPr(mxGetField(prhs[1], 0, "extra_solves"));
    else
      extra_solves = 0;
    if (mxGetField(prhs[1], 0, "prepare_for_c") != NULL)
      prepare_for_c = *mxGetPr(mxGetField(prhs[1], 0, "prepare_for_c"));
  }
  valid_vars = 0;
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "A_0");
  if (xm == NULL) {
    printf("could not find params.A_0.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 8))) {
      printf("A_0 must be size (8,8), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter A_0 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter A_0 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter A_0 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.A_0;
      src = mxGetPr(xm);
      for (i = 0; i < 64; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "A_1");
  if (xm == NULL) {
    /* Attempt to pull A_1 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "A");
    if (cell != NULL)
      xm = mxGetCell(cell, 0);
  }
  if (xm == NULL) {
    printf("could not find params.A_1 or params.A{1}.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 8))) {
      printf("A_1 must be size (8,8), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter A_1 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter A_1 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter A_1 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.A_1;
      src = mxGetPr(xm);
      for (i = 0; i < 64; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "A_2");
  if (xm == NULL) {
    /* Attempt to pull A_2 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "A");
    if (cell != NULL)
      xm = mxGetCell(cell, 1);
  }
  if (xm == NULL) {
    printf("could not find params.A_2 or params.A{2}.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 8))) {
      printf("A_2 must be size (8,8), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter A_2 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter A_2 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter A_2 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.A_2;
      src = mxGetPr(xm);
      for (i = 0; i < 64; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "A_3");
  if (xm == NULL) {
    /* Attempt to pull A_3 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "A");
    if (cell != NULL)
      xm = mxGetCell(cell, 2);
  }
  if (xm == NULL) {
    printf("could not find params.A_3 or params.A{3}.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 8))) {
      printf("A_3 must be size (8,8), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter A_3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter A_3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter A_3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.A_3;
      src = mxGetPr(xm);
      for (i = 0; i < 64; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "A_4");
  if (xm == NULL) {
    /* Attempt to pull A_4 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "A");
    if (cell != NULL)
      xm = mxGetCell(cell, 3);
  }
  if (xm == NULL) {
    printf("could not find params.A_4 or params.A{4}.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 8))) {
      printf("A_4 must be size (8,8), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter A_4 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter A_4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter A_4 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.A_4;
      src = mxGetPr(xm);
      for (i = 0; i < 64; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "A_5");
  if (xm == NULL) {
    /* Attempt to pull A_5 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "A");
    if (cell != NULL)
      xm = mxGetCell(cell, 4);
  }
  if (xm == NULL) {
    printf("could not find params.A_5 or params.A{5}.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 8))) {
      printf("A_5 must be size (8,8), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter A_5 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter A_5 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter A_5 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.A_5;
      src = mxGetPr(xm);
      for (i = 0; i < 64; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "A_6");
  if (xm == NULL) {
    /* Attempt to pull A_6 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "A");
    if (cell != NULL)
      xm = mxGetCell(cell, 5);
  }
  if (xm == NULL) {
    printf("could not find params.A_6 or params.A{6}.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 8))) {
      printf("A_6 must be size (8,8), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter A_6 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter A_6 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter A_6 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.A_6;
      src = mxGetPr(xm);
      for (i = 0; i < 64; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "A_7");
  if (xm == NULL) {
    /* Attempt to pull A_7 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "A");
    if (cell != NULL)
      xm = mxGetCell(cell, 6);
  }
  if (xm == NULL) {
    printf("could not find params.A_7 or params.A{7}.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 8))) {
      printf("A_7 must be size (8,8), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter A_7 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter A_7 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter A_7 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.A_7;
      src = mxGetPr(xm);
      for (i = 0; i < 64; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "A_8");
  if (xm == NULL) {
    /* Attempt to pull A_8 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "A");
    if (cell != NULL)
      xm = mxGetCell(cell, 7);
  }
  if (xm == NULL) {
    printf("could not find params.A_8 or params.A{8}.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 8))) {
      printf("A_8 must be size (8,8), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter A_8 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter A_8 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter A_8 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.A_8;
      src = mxGetPr(xm);
      for (i = 0; i < 64; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "A_9");
  if (xm == NULL) {
    /* Attempt to pull A_9 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "A");
    if (cell != NULL)
      xm = mxGetCell(cell, 8);
  }
  if (xm == NULL) {
    printf("could not find params.A_9 or params.A{9}.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 8))) {
      printf("A_9 must be size (8,8), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter A_9 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter A_9 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter A_9 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.A_9;
      src = mxGetPr(xm);
      for (i = 0; i < 64; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "A_10");
  if (xm == NULL) {
    /* Attempt to pull A_10 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "A");
    if (cell != NULL)
      xm = mxGetCell(cell, 9);
  }
  if (xm == NULL) {
    printf("could not find params.A_10 or params.A{10}.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 8))) {
      printf("A_10 must be size (8,8), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter A_10 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter A_10 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter A_10 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.A_10;
      src = mxGetPr(xm);
      for (i = 0; i < 64; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "A_11");
  if (xm == NULL) {
    /* Attempt to pull A_11 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "A");
    if (cell != NULL)
      xm = mxGetCell(cell, 10);
  }
  if (xm == NULL) {
    printf("could not find params.A_11 or params.A{11}.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 8))) {
      printf("A_11 must be size (8,8), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter A_11 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter A_11 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter A_11 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.A_11;
      src = mxGetPr(xm);
      for (i = 0; i < 64; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "A_12");
  if (xm == NULL) {
    /* Attempt to pull A_12 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "A");
    if (cell != NULL)
      xm = mxGetCell(cell, 11);
  }
  if (xm == NULL) {
    printf("could not find params.A_12 or params.A{12}.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 8))) {
      printf("A_12 must be size (8,8), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter A_12 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter A_12 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter A_12 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.A_12;
      src = mxGetPr(xm);
      for (i = 0; i < 64; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "B_0");
  if (xm == NULL) {
    printf("could not find params.B_0.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 2))) {
      printf("B_0 must be size (8,2), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter B_0 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter B_0 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter B_0 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.B_0;
      src = mxGetPr(xm);
      for (i = 0; i < 16; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "B_1");
  if (xm == NULL) {
    /* Attempt to pull B_1 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "B");
    if (cell != NULL)
      xm = mxGetCell(cell, 0);
  }
  if (xm == NULL) {
    printf("could not find params.B_1 or params.B{1}.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 2))) {
      printf("B_1 must be size (8,2), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter B_1 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter B_1 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter B_1 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.B_1;
      src = mxGetPr(xm);
      for (i = 0; i < 16; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "B_2");
  if (xm == NULL) {
    /* Attempt to pull B_2 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "B");
    if (cell != NULL)
      xm = mxGetCell(cell, 1);
  }
  if (xm == NULL) {
    printf("could not find params.B_2 or params.B{2}.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 2))) {
      printf("B_2 must be size (8,2), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter B_2 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter B_2 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter B_2 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.B_2;
      src = mxGetPr(xm);
      for (i = 0; i < 16; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "B_3");
  if (xm == NULL) {
    /* Attempt to pull B_3 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "B");
    if (cell != NULL)
      xm = mxGetCell(cell, 2);
  }
  if (xm == NULL) {
    printf("could not find params.B_3 or params.B{3}.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 2))) {
      printf("B_3 must be size (8,2), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter B_3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter B_3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter B_3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.B_3;
      src = mxGetPr(xm);
      for (i = 0; i < 16; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "B_4");
  if (xm == NULL) {
    /* Attempt to pull B_4 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "B");
    if (cell != NULL)
      xm = mxGetCell(cell, 3);
  }
  if (xm == NULL) {
    printf("could not find params.B_4 or params.B{4}.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 2))) {
      printf("B_4 must be size (8,2), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter B_4 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter B_4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter B_4 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.B_4;
      src = mxGetPr(xm);
      for (i = 0; i < 16; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "B_5");
  if (xm == NULL) {
    /* Attempt to pull B_5 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "B");
    if (cell != NULL)
      xm = mxGetCell(cell, 4);
  }
  if (xm == NULL) {
    printf("could not find params.B_5 or params.B{5}.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 2))) {
      printf("B_5 must be size (8,2), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter B_5 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter B_5 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter B_5 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.B_5;
      src = mxGetPr(xm);
      for (i = 0; i < 16; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "B_6");
  if (xm == NULL) {
    /* Attempt to pull B_6 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "B");
    if (cell != NULL)
      xm = mxGetCell(cell, 5);
  }
  if (xm == NULL) {
    printf("could not find params.B_6 or params.B{6}.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 2))) {
      printf("B_6 must be size (8,2), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter B_6 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter B_6 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter B_6 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.B_6;
      src = mxGetPr(xm);
      for (i = 0; i < 16; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "B_7");
  if (xm == NULL) {
    /* Attempt to pull B_7 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "B");
    if (cell != NULL)
      xm = mxGetCell(cell, 6);
  }
  if (xm == NULL) {
    printf("could not find params.B_7 or params.B{7}.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 2))) {
      printf("B_7 must be size (8,2), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter B_7 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter B_7 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter B_7 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.B_7;
      src = mxGetPr(xm);
      for (i = 0; i < 16; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "B_8");
  if (xm == NULL) {
    /* Attempt to pull B_8 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "B");
    if (cell != NULL)
      xm = mxGetCell(cell, 7);
  }
  if (xm == NULL) {
    printf("could not find params.B_8 or params.B{8}.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 2))) {
      printf("B_8 must be size (8,2), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter B_8 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter B_8 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter B_8 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.B_8;
      src = mxGetPr(xm);
      for (i = 0; i < 16; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "B_9");
  if (xm == NULL) {
    /* Attempt to pull B_9 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "B");
    if (cell != NULL)
      xm = mxGetCell(cell, 8);
  }
  if (xm == NULL) {
    printf("could not find params.B_9 or params.B{9}.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 2))) {
      printf("B_9 must be size (8,2), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter B_9 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter B_9 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter B_9 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.B_9;
      src = mxGetPr(xm);
      for (i = 0; i < 16; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "B_10");
  if (xm == NULL) {
    /* Attempt to pull B_10 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "B");
    if (cell != NULL)
      xm = mxGetCell(cell, 9);
  }
  if (xm == NULL) {
    printf("could not find params.B_10 or params.B{10}.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 2))) {
      printf("B_10 must be size (8,2), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter B_10 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter B_10 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter B_10 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.B_10;
      src = mxGetPr(xm);
      for (i = 0; i < 16; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "B_11");
  if (xm == NULL) {
    /* Attempt to pull B_11 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "B");
    if (cell != NULL)
      xm = mxGetCell(cell, 10);
  }
  if (xm == NULL) {
    printf("could not find params.B_11 or params.B{11}.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 2))) {
      printf("B_11 must be size (8,2), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter B_11 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter B_11 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter B_11 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.B_11;
      src = mxGetPr(xm);
      for (i = 0; i < 16; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "B_12");
  if (xm == NULL) {
    /* Attempt to pull B_12 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "B");
    if (cell != NULL)
      xm = mxGetCell(cell, 11);
  }
  if (xm == NULL) {
    printf("could not find params.B_12 or params.B{12}.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 2))) {
      printf("B_12 must be size (8,2), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter B_12 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter B_12 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter B_12 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.B_12;
      src = mxGetPr(xm);
      for (i = 0; i < 16; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Q");
  if (xm == NULL) {
    printf("could not find params.Q.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 8))) {
      printf("Q must be size (8,8), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Q must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Q must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Q must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Q;
      src = mxGetPr(xm);
      for (i = 0; i < 64; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "QT");
  if (xm == NULL) {
    printf("could not find params.QT.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 8))) {
      printf("QT must be size (8,8), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter QT must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter QT must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter QT must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.QT;
      src = mxGetPr(xm);
      for (i = 0; i < 64; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "R");
  if (xm == NULL) {
    printf("could not find params.R.\n");
  } else {
    if (!((mxGetM(xm) == 2) && (mxGetN(xm) == 2))) {
      printf("R must be size (2,2), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter R must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter R must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter R must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.R;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "d_0");
  if (xm == NULL) {
    printf("could not find params.d_0.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 1))) {
      printf("d_0 must be size (8,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter d_0 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter d_0 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter d_0 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.d_0;
      src = mxGetPr(xm);
      for (i = 0; i < 8; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "d_1");
  if (xm == NULL) {
    /* Attempt to pull d_1 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "d");
    if (cell != NULL)
      xm = mxGetCell(cell, 0);
  }
  if (xm == NULL) {
    printf("could not find params.d_1 or params.d{1}.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 1))) {
      printf("d_1 must be size (8,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter d_1 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter d_1 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter d_1 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.d_1;
      src = mxGetPr(xm);
      for (i = 0; i < 8; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "d_2");
  if (xm == NULL) {
    /* Attempt to pull d_2 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "d");
    if (cell != NULL)
      xm = mxGetCell(cell, 1);
  }
  if (xm == NULL) {
    printf("could not find params.d_2 or params.d{2}.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 1))) {
      printf("d_2 must be size (8,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter d_2 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter d_2 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter d_2 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.d_2;
      src = mxGetPr(xm);
      for (i = 0; i < 8; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "d_3");
  if (xm == NULL) {
    /* Attempt to pull d_3 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "d");
    if (cell != NULL)
      xm = mxGetCell(cell, 2);
  }
  if (xm == NULL) {
    printf("could not find params.d_3 or params.d{3}.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 1))) {
      printf("d_3 must be size (8,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter d_3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter d_3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter d_3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.d_3;
      src = mxGetPr(xm);
      for (i = 0; i < 8; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "d_4");
  if (xm == NULL) {
    /* Attempt to pull d_4 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "d");
    if (cell != NULL)
      xm = mxGetCell(cell, 3);
  }
  if (xm == NULL) {
    printf("could not find params.d_4 or params.d{4}.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 1))) {
      printf("d_4 must be size (8,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter d_4 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter d_4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter d_4 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.d_4;
      src = mxGetPr(xm);
      for (i = 0; i < 8; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "d_5");
  if (xm == NULL) {
    /* Attempt to pull d_5 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "d");
    if (cell != NULL)
      xm = mxGetCell(cell, 4);
  }
  if (xm == NULL) {
    printf("could not find params.d_5 or params.d{5}.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 1))) {
      printf("d_5 must be size (8,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter d_5 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter d_5 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter d_5 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.d_5;
      src = mxGetPr(xm);
      for (i = 0; i < 8; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "d_6");
  if (xm == NULL) {
    /* Attempt to pull d_6 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "d");
    if (cell != NULL)
      xm = mxGetCell(cell, 5);
  }
  if (xm == NULL) {
    printf("could not find params.d_6 or params.d{6}.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 1))) {
      printf("d_6 must be size (8,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter d_6 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter d_6 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter d_6 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.d_6;
      src = mxGetPr(xm);
      for (i = 0; i < 8; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "d_7");
  if (xm == NULL) {
    /* Attempt to pull d_7 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "d");
    if (cell != NULL)
      xm = mxGetCell(cell, 6);
  }
  if (xm == NULL) {
    printf("could not find params.d_7 or params.d{7}.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 1))) {
      printf("d_7 must be size (8,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter d_7 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter d_7 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter d_7 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.d_7;
      src = mxGetPr(xm);
      for (i = 0; i < 8; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "d_8");
  if (xm == NULL) {
    /* Attempt to pull d_8 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "d");
    if (cell != NULL)
      xm = mxGetCell(cell, 7);
  }
  if (xm == NULL) {
    printf("could not find params.d_8 or params.d{8}.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 1))) {
      printf("d_8 must be size (8,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter d_8 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter d_8 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter d_8 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.d_8;
      src = mxGetPr(xm);
      for (i = 0; i < 8; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "d_9");
  if (xm == NULL) {
    /* Attempt to pull d_9 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "d");
    if (cell != NULL)
      xm = mxGetCell(cell, 8);
  }
  if (xm == NULL) {
    printf("could not find params.d_9 or params.d{9}.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 1))) {
      printf("d_9 must be size (8,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter d_9 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter d_9 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter d_9 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.d_9;
      src = mxGetPr(xm);
      for (i = 0; i < 8; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "d_10");
  if (xm == NULL) {
    /* Attempt to pull d_10 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "d");
    if (cell != NULL)
      xm = mxGetCell(cell, 9);
  }
  if (xm == NULL) {
    printf("could not find params.d_10 or params.d{10}.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 1))) {
      printf("d_10 must be size (8,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter d_10 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter d_10 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter d_10 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.d_10;
      src = mxGetPr(xm);
      for (i = 0; i < 8; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "d_11");
  if (xm == NULL) {
    /* Attempt to pull d_11 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "d");
    if (cell != NULL)
      xm = mxGetCell(cell, 10);
  }
  if (xm == NULL) {
    printf("could not find params.d_11 or params.d{11}.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 1))) {
      printf("d_11 must be size (8,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter d_11 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter d_11 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter d_11 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.d_11;
      src = mxGetPr(xm);
      for (i = 0; i < 8; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "d_12");
  if (xm == NULL) {
    /* Attempt to pull d_12 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "d");
    if (cell != NULL)
      xm = mxGetCell(cell, 11);
  }
  if (xm == NULL) {
    printf("could not find params.d_12 or params.d{12}.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 1))) {
      printf("d_12 must be size (8,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter d_12 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter d_12 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter d_12 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.d_12;
      src = mxGetPr(xm);
      for (i = 0; i < 8; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "half_road_width");
  if (xm == NULL) {
    printf("could not find params.half_road_width.\n");
  } else {
    if (!((mxGetM(xm) == 1) && (mxGetN(xm) == 1))) {
      printf("half_road_width must be size (1,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter half_road_width must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter half_road_width must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter half_road_width must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.half_road_width;
      src = mxGetPr(xm);
      for (i = 0; i < 1; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "target");
  if (xm == NULL) {
    printf("could not find params.target.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 1))) {
      printf("target must be size (8,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter target must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter target must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter target must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.target;
      src = mxGetPr(xm);
      for (i = 0; i < 8; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "umax");
  if (xm == NULL) {
    printf("could not find params.umax.\n");
  } else {
    if (!((mxGetM(xm) == 2) && (mxGetN(xm) == 1))) {
      printf("umax must be size (2,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter umax must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter umax must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter umax must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.umax;
      src = mxGetPr(xm);
      for (i = 0; i < 2; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "x_0");
  if (xm == NULL) {
    printf("could not find params.x_0.\n");
  } else {
    if (!((mxGetM(xm) == 8) && (mxGetN(xm) == 1))) {
      printf("x_0 must be size (8,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter x_0 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter x_0 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter x_0 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.x_0;
      src = mxGetPr(xm);
      for (i = 0; i < 8; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  if (valid_vars != 46) {
    printf("Error: %d parameters are invalid.\n", 46 - valid_vars);
    mexErrMsgTxt("invalid parameters found.");
  }
  if (prepare_for_c) {
    printf("settings.prepare_for_c == 1. thus, outputting for C.\n");
    for (i = 0; i < 8; i++)
      printf("  params.x_0[%d] = %.6g;\n", i, params.x_0[i]);
    for (i = 0; i < 8; i++)
      printf("  params.target[%d] = %.6g;\n", i, params.target[i]);
    for (i = 0; i < 64; i++)
      printf("  params.Q[%d] = %.6g;\n", i, params.Q[i]);
    for (i = 0; i < 4; i++)
      printf("  params.R[%d] = %.6g;\n", i, params.R[i]);
    for (i = 0; i < 64; i++)
      printf("  params.QT[%d] = %.6g;\n", i, params.QT[i]);
    for (i = 0; i < 64; i++)
      printf("  params.A_0[%d] = %.6g;\n", i, params.A_0[i]);
    for (i = 0; i < 16; i++)
      printf("  params.B_0[%d] = %.6g;\n", i, params.B_0[i]);
    for (i = 0; i < 8; i++)
      printf("  params.d_0[%d] = %.6g;\n", i, params.d_0[i]);
    for (i = 0; i < 64; i++)
      printf("  params.A_1[%d] = %.6g;\n", i, params.A_1[i]);
    for (i = 0; i < 16; i++)
      printf("  params.B_1[%d] = %.6g;\n", i, params.B_1[i]);
    for (i = 0; i < 8; i++)
      printf("  params.d_1[%d] = %.6g;\n", i, params.d_1[i]);
    for (i = 0; i < 64; i++)
      printf("  params.A_2[%d] = %.6g;\n", i, params.A_2[i]);
    for (i = 0; i < 16; i++)
      printf("  params.B_2[%d] = %.6g;\n", i, params.B_2[i]);
    for (i = 0; i < 8; i++)
      printf("  params.d_2[%d] = %.6g;\n", i, params.d_2[i]);
    for (i = 0; i < 64; i++)
      printf("  params.A_3[%d] = %.6g;\n", i, params.A_3[i]);
    for (i = 0; i < 16; i++)
      printf("  params.B_3[%d] = %.6g;\n", i, params.B_3[i]);
    for (i = 0; i < 8; i++)
      printf("  params.d_3[%d] = %.6g;\n", i, params.d_3[i]);
    for (i = 0; i < 64; i++)
      printf("  params.A_4[%d] = %.6g;\n", i, params.A_4[i]);
    for (i = 0; i < 16; i++)
      printf("  params.B_4[%d] = %.6g;\n", i, params.B_4[i]);
    for (i = 0; i < 8; i++)
      printf("  params.d_4[%d] = %.6g;\n", i, params.d_4[i]);
    for (i = 0; i < 64; i++)
      printf("  params.A_5[%d] = %.6g;\n", i, params.A_5[i]);
    for (i = 0; i < 16; i++)
      printf("  params.B_5[%d] = %.6g;\n", i, params.B_5[i]);
    for (i = 0; i < 8; i++)
      printf("  params.d_5[%d] = %.6g;\n", i, params.d_5[i]);
    for (i = 0; i < 64; i++)
      printf("  params.A_6[%d] = %.6g;\n", i, params.A_6[i]);
    for (i = 0; i < 16; i++)
      printf("  params.B_6[%d] = %.6g;\n", i, params.B_6[i]);
    for (i = 0; i < 8; i++)
      printf("  params.d_6[%d] = %.6g;\n", i, params.d_6[i]);
    for (i = 0; i < 64; i++)
      printf("  params.A_7[%d] = %.6g;\n", i, params.A_7[i]);
    for (i = 0; i < 16; i++)
      printf("  params.B_7[%d] = %.6g;\n", i, params.B_7[i]);
    for (i = 0; i < 8; i++)
      printf("  params.d_7[%d] = %.6g;\n", i, params.d_7[i]);
    for (i = 0; i < 64; i++)
      printf("  params.A_8[%d] = %.6g;\n", i, params.A_8[i]);
    for (i = 0; i < 16; i++)
      printf("  params.B_8[%d] = %.6g;\n", i, params.B_8[i]);
    for (i = 0; i < 8; i++)
      printf("  params.d_8[%d] = %.6g;\n", i, params.d_8[i]);
    for (i = 0; i < 64; i++)
      printf("  params.A_9[%d] = %.6g;\n", i, params.A_9[i]);
    for (i = 0; i < 16; i++)
      printf("  params.B_9[%d] = %.6g;\n", i, params.B_9[i]);
    for (i = 0; i < 8; i++)
      printf("  params.d_9[%d] = %.6g;\n", i, params.d_9[i]);
    for (i = 0; i < 64; i++)
      printf("  params.A_10[%d] = %.6g;\n", i, params.A_10[i]);
    for (i = 0; i < 16; i++)
      printf("  params.B_10[%d] = %.6g;\n", i, params.B_10[i]);
    for (i = 0; i < 8; i++)
      printf("  params.d_10[%d] = %.6g;\n", i, params.d_10[i]);
    for (i = 0; i < 64; i++)
      printf("  params.A_11[%d] = %.6g;\n", i, params.A_11[i]);
    for (i = 0; i < 16; i++)
      printf("  params.B_11[%d] = %.6g;\n", i, params.B_11[i]);
    for (i = 0; i < 8; i++)
      printf("  params.d_11[%d] = %.6g;\n", i, params.d_11[i]);
    for (i = 0; i < 64; i++)
      printf("  params.A_12[%d] = %.6g;\n", i, params.A_12[i]);
    for (i = 0; i < 16; i++)
      printf("  params.B_12[%d] = %.6g;\n", i, params.B_12[i]);
    for (i = 0; i < 8; i++)
      printf("  params.d_12[%d] = %.6g;\n", i, params.d_12[i]);
    for (i = 0; i < 1; i++)
      printf("  params.half_road_width[%d] = %.6g;\n", i, params.half_road_width[i]);
    for (i = 0; i < 2; i++)
      printf("  params.umax[%d] = %.6g;\n", i, params.umax[i]);
  }
  /* Perform the actual solve in here. */
  steps = solve();
  /* For profiling purposes, allow extra silent solves if desired. */
  settings.verbose = 0;
  for (i = 0; i < extra_solves; i++)
    solve();
  /* Update the status variables. */
  plhs[1] = mxCreateStructArray(1, dims1x1of1, 4, status_names);
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "optval", xm);
  *mxGetPr(xm) = work.optval;
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "gap", xm);
  *mxGetPr(xm) = work.gap;
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "steps", xm);
  *mxGetPr(xm) = steps;
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "converged", xm);
  *mxGetPr(xm) = work.converged;
  /* Extract variable values. */
  plhs[0] = mxCreateStructArray(1, dims1x1of1, num_var_names, var_names);
  /* Create cell arrays for indexed variables. */
  dims[0] = 12;
  cell = mxCreateCellArray(1, dims);
  mxSetField(plhs[0], 0, "u", cell);
  dims[0] = 13;
  cell = mxCreateCellArray(1, dims);
  mxSetField(plhs[0], 0, "x", cell);
  xm = mxCreateDoubleMatrix(2, 1, mxREAL);
  mxSetField(plhs[0], 0, "u_0", xm);
  dest = mxGetPr(xm);
  src = vars.u_0;
  for (i = 0; i < 2; i++) {
    *dest++ = *src++;
  }
  xm = mxCreateDoubleMatrix(2, 1, mxREAL);
  mxSetField(plhs[0], 0, "u_1", xm);
  xm_cell = mxCreateDoubleMatrix(2, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "u");
  mxSetCell(cell, 0, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.u_1;
  for (i = 0; i < 2; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(2, 1, mxREAL);
  mxSetField(plhs[0], 0, "u_2", xm);
  xm_cell = mxCreateDoubleMatrix(2, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "u");
  mxSetCell(cell, 1, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.u_2;
  for (i = 0; i < 2; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(2, 1, mxREAL);
  mxSetField(plhs[0], 0, "u_3", xm);
  xm_cell = mxCreateDoubleMatrix(2, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "u");
  mxSetCell(cell, 2, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.u_3;
  for (i = 0; i < 2; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(2, 1, mxREAL);
  mxSetField(plhs[0], 0, "u_4", xm);
  xm_cell = mxCreateDoubleMatrix(2, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "u");
  mxSetCell(cell, 3, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.u_4;
  for (i = 0; i < 2; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(2, 1, mxREAL);
  mxSetField(plhs[0], 0, "u_5", xm);
  xm_cell = mxCreateDoubleMatrix(2, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "u");
  mxSetCell(cell, 4, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.u_5;
  for (i = 0; i < 2; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(2, 1, mxREAL);
  mxSetField(plhs[0], 0, "u_6", xm);
  xm_cell = mxCreateDoubleMatrix(2, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "u");
  mxSetCell(cell, 5, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.u_6;
  for (i = 0; i < 2; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(2, 1, mxREAL);
  mxSetField(plhs[0], 0, "u_7", xm);
  xm_cell = mxCreateDoubleMatrix(2, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "u");
  mxSetCell(cell, 6, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.u_7;
  for (i = 0; i < 2; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(2, 1, mxREAL);
  mxSetField(plhs[0], 0, "u_8", xm);
  xm_cell = mxCreateDoubleMatrix(2, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "u");
  mxSetCell(cell, 7, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.u_8;
  for (i = 0; i < 2; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(2, 1, mxREAL);
  mxSetField(plhs[0], 0, "u_9", xm);
  xm_cell = mxCreateDoubleMatrix(2, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "u");
  mxSetCell(cell, 8, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.u_9;
  for (i = 0; i < 2; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(2, 1, mxREAL);
  mxSetField(plhs[0], 0, "u_10", xm);
  xm_cell = mxCreateDoubleMatrix(2, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "u");
  mxSetCell(cell, 9, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.u_10;
  for (i = 0; i < 2; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(2, 1, mxREAL);
  mxSetField(plhs[0], 0, "u_11", xm);
  xm_cell = mxCreateDoubleMatrix(2, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "u");
  mxSetCell(cell, 10, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.u_11;
  for (i = 0; i < 2; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(2, 1, mxREAL);
  mxSetField(plhs[0], 0, "u_12", xm);
  xm_cell = mxCreateDoubleMatrix(2, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "u");
  mxSetCell(cell, 11, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.u_12;
  for (i = 0; i < 2; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(8, 1, mxREAL);
  mxSetField(plhs[0], 0, "x_1", xm);
  xm_cell = mxCreateDoubleMatrix(8, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "x");
  mxSetCell(cell, 0, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.x_1;
  for (i = 0; i < 8; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(8, 1, mxREAL);
  mxSetField(plhs[0], 0, "x_2", xm);
  xm_cell = mxCreateDoubleMatrix(8, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "x");
  mxSetCell(cell, 1, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.x_2;
  for (i = 0; i < 8; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(8, 1, mxREAL);
  mxSetField(plhs[0], 0, "x_3", xm);
  xm_cell = mxCreateDoubleMatrix(8, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "x");
  mxSetCell(cell, 2, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.x_3;
  for (i = 0; i < 8; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(8, 1, mxREAL);
  mxSetField(plhs[0], 0, "x_4", xm);
  xm_cell = mxCreateDoubleMatrix(8, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "x");
  mxSetCell(cell, 3, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.x_4;
  for (i = 0; i < 8; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(8, 1, mxREAL);
  mxSetField(plhs[0], 0, "x_5", xm);
  xm_cell = mxCreateDoubleMatrix(8, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "x");
  mxSetCell(cell, 4, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.x_5;
  for (i = 0; i < 8; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(8, 1, mxREAL);
  mxSetField(plhs[0], 0, "x_6", xm);
  xm_cell = mxCreateDoubleMatrix(8, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "x");
  mxSetCell(cell, 5, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.x_6;
  for (i = 0; i < 8; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(8, 1, mxREAL);
  mxSetField(plhs[0], 0, "x_7", xm);
  xm_cell = mxCreateDoubleMatrix(8, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "x");
  mxSetCell(cell, 6, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.x_7;
  for (i = 0; i < 8; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(8, 1, mxREAL);
  mxSetField(plhs[0], 0, "x_8", xm);
  xm_cell = mxCreateDoubleMatrix(8, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "x");
  mxSetCell(cell, 7, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.x_8;
  for (i = 0; i < 8; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(8, 1, mxREAL);
  mxSetField(plhs[0], 0, "x_9", xm);
  xm_cell = mxCreateDoubleMatrix(8, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "x");
  mxSetCell(cell, 8, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.x_9;
  for (i = 0; i < 8; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(8, 1, mxREAL);
  mxSetField(plhs[0], 0, "x_10", xm);
  xm_cell = mxCreateDoubleMatrix(8, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "x");
  mxSetCell(cell, 9, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.x_10;
  for (i = 0; i < 8; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(8, 1, mxREAL);
  mxSetField(plhs[0], 0, "x_11", xm);
  xm_cell = mxCreateDoubleMatrix(8, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "x");
  mxSetCell(cell, 10, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.x_11;
  for (i = 0; i < 8; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(8, 1, mxREAL);
  mxSetField(plhs[0], 0, "x_12", xm);
  xm_cell = mxCreateDoubleMatrix(8, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "x");
  mxSetCell(cell, 11, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.x_12;
  for (i = 0; i < 8; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(8, 1, mxREAL);
  mxSetField(plhs[0], 0, "x_13", xm);
  xm_cell = mxCreateDoubleMatrix(8, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "x");
  mxSetCell(cell, 12, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.x_13;
  for (i = 0; i < 8; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
}
