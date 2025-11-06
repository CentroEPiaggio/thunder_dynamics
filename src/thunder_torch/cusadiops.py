# Copyright (c) 2012-2024 Se Hwan Jeon and others

# Permission is hereby granted, free of charge, to any person obtaining
# a copy of this software and associated documentation files (the
# "Software"), to deal in the Software without restriction, including
# without limitation the rights to use, copy, modify, merge, publish,
# distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to
# the following conditions:

# The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
# LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
# OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
# WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

# Taken from https://github.com/se-hwan/cusadi/blob/dev/src/CusadiOperations.py


from casadi import * 

OP_PYTORCH_DICT = {
    OP_ASSIGN:              "\n        work[:, %d] = work[:, %d]",
    OP_ADD:                 "\n        work[:, %d] = work[:, %d] + work[:, %d]",
    OP_SUB:                 "\n        work[:, %d] = work[:, %d] - work[:, %d]",
    OP_MUL:                 "\n        work[:, %d] = work[:, %d] * work[:, %d]",
    OP_DIV:                 "\n        work[:, %d] = work[:, %d] / work[:, %d]",
    OP_NEG:                 "\n        work[:, %d] = -work[:, %d]",
    OP_EXP:                 "\n        work[:, %d] = torch.exp(work[:, %d])",
    OP_LOG:                 "\n        work[:, %d] = torch.log(work[:, %d])",
    OP_POW:                 "\n        work[:, %d] = torch.pow(work[:, %d], work[:, %d])",
    OP_CONSTPOW:            "\n        work[:, %d] = torch.pow(work[:, %d], work[:, %d])",
    OP_SQRT:                "\n        work[:, %d] = torch.sqrt(work[:, %d])",
    OP_SQ:                  "\n        work[:, %d] = work[:, %d] * work[:, %d]",
    OP_TWICE:               "\n        work[:, %d] = 2.*(work[:, %d])",
    OP_SIN:                 "\n        work[:, %d] = torch.sin(work[:, %d])",
    OP_COS:                 "\n        work[:, %d] = torch.cos(work[:, %d])",
    OP_TAN:                 "\n        work[:, %d] = torch.tan(work[:, %d])",
    OP_ASIN:                "\n        work[:, %d] = torch.asin(work[:, %d])",
    OP_ACOS:                "\n        work[:, %d] = torch.acos(work[:, %d])",
    OP_ATAN:                "\n        work[:, %d] = torch.atan(work[:, %d])",
    OP_LT:                  "\n        work[:, %d] = work[:, %d] < work[:, %d]",
    OP_LE:                  "\n        work[:, %d] = work[:, %d] <= work[:, %d]",
    OP_EQ:                  "\n        work[:, %d] = work[:, %d] == work[:, %d]",
    OP_NE:                  "\n        work[:, %d] = work[:, %d] != work[:, %d]",
    OP_NOT:                 "\n        work[:, %d] = torch.logical_not(work[:, %d])",
    OP_AND:                 "\n        work[:, %d] = torch.logical_and(work[:, %d], work[:, %d])",
    OP_OR:                  "\n        work[:, %d] = torch.logical_or(work[:, %d], work[:, %d])",
    OP_FLOOR:               "\n        work[:, %d] = torch.floor(work[:, %d])",
    OP_CEIL:                "\n        work[:, %d] = torch.ceil(work[:, %d])",
    OP_FMOD:                "\n        work[:, %d] = torch.fmod(work[:, %d], work[:, %d])",
    OP_FABS:                "\n        work[:, %d] = torch.abs(work[:, %d])",
    OP_SIGN:                "\n        work[:, %d] = torch.sign(work[:, %d])",
    OP_COPYSIGN:            "\n        work[:, %d] = torch.copysign(work[:, %d], work[:, %d])",
    OP_IF_ELSE_ZERO:        "\n        work[:, %d] = (work[:, %d] == 0) ? 0 : work[:, %d]",
    OP_ERF:                 "\n        work[:, %d] = torch.erf(work[:, %d])",
    OP_FMIN:                "\n        work[:, %d] = torch.fmin(work[:, %d], work[:, %d])",
    OP_FMAX:                "\n        work[:, %d] = torch.fmax(work[:, %d], work[:, %d])",
    OP_INV:                 "\n        work[:, %d] = 1./(work[:, %d])",
    OP_SINH:                "\n        work[:, %d] = torch.sinh(work[:, %d])",
    OP_COSH:                "\n        work[:, %d] = torch.cosh(work[:, %d])",
    OP_TANH:                "\n        work[:, %d] = torch.tanh(work[:, %d])",
    OP_ASINH:               "\n        work[:, %d] = torch.asinh(work[:, %d])",
    OP_ACOSH:               "\n        work[:, %d] = torch.acosh(work[:, %d])",
    OP_ATANH:               "\n        work[:, %d] = torch.atanh(work[:, %d])",
    OP_ATAN2:               "\n        work[:, %d] = torch.atan2(work[:, %d], work[:, %d])",
    OP_CONST:               "\n        work[:, %d] = %f",
    OP_INPUT:               "\n        work[:, %d] = inputs[%d][:, %d]",
    OP_OUTPUT:              "\n        outputs[%d][:, %d] = work[:, %d]",
}
