#
# Copyright (C) 2023 - This file is part of IPECC project
#
# Authors:
#     Karim KHALFALLAH <karim.khalfallah@ssi.gouv.fr>
#     Ryad BENADJILA <ryadbenadjila@gmail.com>
#
# Contributors:
#     Adrian THILLARD
#     Emmanuel PROUFF

# Routine replacing addition when scalar bit is 0 (naive implem).
.zadd_voidL:
.zadd_voidL_export:
.zadd_voidL_dbg:
	NOP
	STOP

#####################################################################
#               C O Z   D O U B L E   &   U P D A T E
#####################################################################
.zdbl_not_alwaysL:
.zdbl_not_alwaysL_export:
# ****************************************************************
# compute ( R0)     (   R0)
#         ( R1)z -> ([2]R1)z'
# ****************************************************************
.zdbl_not_always_op1L_dbg:
  BARRIER
  FPREDC     ZR01    ZR01     N        # N(8) <- Z²
  FPREDC     YR1     YR1      E        # E(9) <- Y²
  BARRIER
  FPREDC     N       N        Nsq0     # Nsq0(23) <- N²
  FPREDC     E       E        L        # L(16) <- E²
  NNADD      E       N        EpN      # EpN(25) <- E + N
  BARRIER
  NNMOV      Nsq0             Nsq      # Nsq(8) <- Nsq0(23)
  FPREDC     XR1     XR1      BZd      # BZd(23) <- X² (clobbers Nsq0)
  NNADD      XR1     E        XpE      # XpE(20) <- X + E
  NNSUB      XpE     twop     red
  NNADD,p5   red     patchme  XpE
  BARRIER
  NNADD      BZd     L        BpL      # BpL(17) <- BZd + L
  NNSUB      BpL     twop     red
  NNADD,p5   red     patchme  BpL
  FPREDC     XpE     XpE      XpE      # XpE(20) <- (X + E)² (clobbers previous X + E)
  NNADD      BZd     BZd      twoB     # twoB(21) <- 2BZd
  NNSUB      twoB    twop     red
  NNADD,p5   red     patchme  twoB
  NNADD      twoB    BZd      threeB   # threeB(23) <- 3BZd
  NNSUB      threeB  twop     red
  NNADD,p5   red     patchme  threeB
  NNSUB      EpN     twop     red
  NNADD,p5   red     patchme  EpN
  NNADD      YR1     ZR01     YpZ      # YpZ(21) <- Y + Z  (clobbers twoB)
  NNSUB      YpZ     twop     red
  NNADD,p5   red     patchme  YpZ
  FPREDC     YpZ     YpZ      YpZsq    # YpZsq(21) <- (Y + Z)²
  BARRIER
  FPREDC     a       Nsq      Nsq      # Nsq(8) <- aN²  (clobbers previous N²)
  NNADD      L       L        L        # L(16) <- 2E² (clobbers previous E²)
  NNSUB      L       twop     red
  NNADD,p5   red     patchme  L
  NNADD      L       L        L        # L(16) <- 4E² (clobbers previous 2E²)
  NNSUB      L       twop     red
  NNADD,p5   red     patchme  L
  NNADD,p22  L       L        L        # L(16) <- 8E² (clobbers previous 4E²)     # used to be __Y_OF_UPDATE__
  NNSUB      L       twop     red
  NNADD,p5   red     patchme  L
  BARRIER
  NNSUB      XpE     BpL      XpE      # XpE(20) <- (X + E)² - BZd - L (clobbers previous (X + E)²)
  NNADD,p5   XpE     patchme  XpE
  NNADD      XpE     XpE      S        # S(17) <- 2((X + E)² - BZd - L) (clobbers previous BZd + L)
  NNSUB      S       twop     red
  NNADD,p5   red     patchme  S
#  NNMOV,p23  S                XR1      # XR1 <- S = 2((X + E)² - BZd - L)          __X_OF_UPDATE__
  BARRIER
  NNSUB      YpZsq   EpN      Ztmp     # Ztmp(25) <- (Y + Z)² - E - N
  NNADD,p5   Ztmp    patchme  Ztmp
  NNMOV,p61  Ztmp             ZR01     # ZR01(26) <- (Y + Z)² - E - N si pt pas de 2-torsion
  NNADD      BZd     Nsq      MD       # M(8) <- 3BZd + aN² (clobbers N=Z²)
  NNSUB      MD      twop     red
  NNADD,p5   red     patchme  MD
  FPREDC     MD      MD       Msq      # Msq(21) <- (3BZd + aN²)² (clobbers YpZsq = (Y + Z)²)
  NNADD      S       S        twoS     # twoS(23) <- 2S (clobbers Nsq = aN²)
  NNSUB      twoS    twop     red
  NNADD,p5   red     patchme  twoS
  BARRIER
  NNSUB,p51  Msq     twoS     XR1      # XR1(6) <- = M² - 2S = (3BZd + aN²)² - 2S  __X_OF_DOUBLE__
  NNADD,p5   XR1     patchme  XR1      # detection = 0 par ,p55 supprimé : seul compte Y = 0 pour détecter DBL=0
  BARRIER
  NNSUB      S       XR1      S        # S(17) <- S - XR1 (clobbers previous 2((X + E)² - BZd - L))
  NNADD,p5   S       patchme  S
  FPREDC     S       MD       S        # S(17) <- M(S - XR1) (clobbers previous S - XR1)
# Back-up YR1 before it's clobbered w/ the Y of double
	NNADD      YR1     YR1      2YR1     # 2YR1(8) <- YR1 + YR1 (clobbers previous M = 3BZd + aN²)
	NNSUB      2YR1    twop     red
	NNADD,p5   red     patchme  2YR1
	FPREDC     2YR1    2YR1     4YR1sq   # 4YR1sq(23) <- 4YR1² (clobbers previous S = M(S - XR1))
	BARRIER
  FPREDC     4YR1sq  XR0      XR0      # XR0 <- XR0 * (2YR1)²   __X_OF_R0_AS_UPDATE__
	FPREDC     4YR1sq  2YR1     8YR1cu   # 8YR1cu(23) <- 8YR1^3 (clobbers previous 4YR1sq = 4YR1²)
	BARRIER
	FPREDC     YR0     8YR1cu   YR0      # YR0 <- YR0 * (2YR1)^3  __Y_OF_R0_AS_UPDATE__

#  BARRIER
  NNSUB,p52  S       L        YR1      # YR1(7) <- M(S - XR0) - 8E²              __Y_OF_DOUBLE__
  NNADD,p5   YR1     patchme  YR1      # ,p56 supprimé ici : detection of a null double done above
	BARRIER
.zdbl_not_always_oplastL_dbg:
	NOP
	STOP
