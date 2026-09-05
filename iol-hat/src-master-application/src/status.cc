#include "osal.h"
#include "osal_log.h"
#include "iolink.h"
#include "iolink_smi.h"
#include "iolink_handler.h"
#include "server.h"
#include "iolink_max14819.h"
#include "iolink_pl.h"

#include "status.h"
#include "common.h"

#include <unistd.h>

uint16_t pdCount0, pdCount1;


void *runStatus (void *_arg)
{	
	uint16_t myTime0;
	uint16_t myTime1;
	
	pdCount0 = 0;
	pdCount1 = 0;
	unsigned zeroSeconds[2] = {0, 0};

  // Start Status thread Server
	while (1)
	{
		
		
		// 0 PD exchanges in the last second: report 0 ms (avoid integer divide by zero).
		uint16_t myCount0 = pdCount0;
		uint16_t myCount1 = pdCount1;
		myTime0 = (myCount0 > 0) ? (1000 / myCount0) : 0;
		myTime1 = (myCount1 > 0) ? (1000 / myCount1) : 0;

		pdCount0 = 0;
		pdCount1 = 0;
		
		
		LOG_INFO (IOLINK_PL_LOG, "Cycle time chn #1=%d ms, chn #2=%d ms\n", myTime0, myTime1);

		// BBB 2026-09-05: wedge detector. In OPERATE the stack exchanges process
		// data every cycle (~12 ms) whether or not any TCP client asks for it, so
		// "port RUNNING but zero PD exchanges for 2 s" means the DL is stuck
		// (seen when a reply interrupt is lost after a retry). Until the DL
		// watchdog recovers it, stop advertising the frozen buffer as valid so the
		// dashboard's stale-data logic can fail closed instead of trusting it.
		for (int p = 0; p < 2; p++)
		{
			uint16_t cnt = (p == 0) ? myCount0 : myCount1;
			if (iolink_app_master.app_port[p].app_port_state == IOL_STATE_RUNNING && cnt == 0)
			{
				if (++zeroSeconds[p] >= 2)
				{
					if (status[p].pdInValid)
					{
						LOG_WARNING (IOLINK_PL_LOG, "Port %d: RUNNING but no process data for %u s - marking PD invalid\n", p + 1, zeroSeconds[p]);
					}
					status[p].pdInValid = 0;
				}
			}
			else
			{
				zeroSeconds[p] = 0;
			}
		}

		sleep(1);
	}
}