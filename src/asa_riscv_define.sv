package riscv_defines;

	parameter XLEN             = 32;
	parameter INSTR_SIZE       = 32;
//////////////////////////////////////////////////////////////////////////////
//      _    _    _   _    ___                       _   _                  //
//     / \  | |  | | | |  / _ \ _ __   ___ _ __ __ _| |_(_) ___  _ __  ___  //
//    / _ \ | |  | | | | | | | | '_ \ / _ \ '__/ _` | __| |/ _ \| '_ \/ __| //
//   / ___ \| |__| |_| | | |_| | |_) |  __/ | | (_| | |_| | (_) | | | \__ \ //
//  /_/   \_\_____\___/   \___/| .__/ \___|_|  \__,_|\__|_|\___/|_| |_|___/ //
//                             |_|                                          //
//////////////////////////////////////////////////////////////////////////////

parameter ALU_OP_WIDTH = 6;
	

/////////////////////////////////////////////////
// ______ _____ _____ ___________ ___________  //
// |  _  \  ___/  __ \  _  |  _  \  ___| ___ \ //
// | | | | |__ | /  \/ | | | | | | |__ | |_/ / //
// | | | |  __|| |   | | | | | | |  __||    /  //
// | |/ /| |___| \__/\ \_/ / |/ /| |___| |\ \  //
// |___/ \____/ \____/\___/|___/ \____/\_| \_| //
/////////////////////////////////////////////////
	parameter [ 6:2] OPC_LOAD     = 5'b00_000;
	parameter [ 6:2] OPC_LOAD_FP  = 5'b00_001;
	parameter [ 6:2] OPC_MISC_MEM = 5'b00_011;
	parameter [ 6:2] OPC_OP_IMM   = 5'b00_100; 
	parameter [ 6:2] OPC_AUIPC    = 5'b00_101;
	parameter [ 6:2] OPC_OP_IMM32 = 5'b00_110;
	parameter [ 6:2] OPC_STORE    = 5'b01_000;
	parameter [ 6:2] OPC_STORE_FP = 5'b01_001;
	parameter [ 6:2] OPC_AMO      = 5'b01_011;
	parameter [ 6:2] OPC_OP       = 5'b01_100;
	parameter [ 6:2] OPC_LUI      = 5'b01_101;
	parameter [ 6:2] OPC_OP32     = 5'b01_110;
	parameter [ 6:2] OPC_MADD     = 5'b10_000;
	parameter [ 6:2] OPC_MSUB     = 5'b10_001;
	parameter [ 6:2] OPC_NMSUB    = 5'b10_010;
	parameter [ 6:2] OPC_NMADD    = 5'b10_011;
	parameter [ 6:2] OPC_OP_FP    = 5'b10_100;
	parameter [ 6:2] OPC_BRANCH   = 5'b11_000;
	parameter [ 6:2] OPC_JALR     = 5'b11_001;
	parameter [ 6:2] OPC_JAL      = 5'b11_011;
	parameter [ 6:2] OPC_SYSTEM   = 5'b11_100;

	parameter PCLEN               = 20;
