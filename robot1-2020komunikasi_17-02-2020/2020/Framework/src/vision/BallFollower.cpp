/*
 *   BallFollower.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <stdio.h>
#include "ImgProcess.h"
#include "MX28.h"
#include "Head.h"
#include "Action.h"
#include "Walking.h"
#include "BallFollower.h"
#include "MotionStatus.h"

//int lagi=1;
int d=-30;int t=0;
using namespace Robot;


BallFollower::BallFollower()
{
	m_NoBallMaxCount = 10;
	m_NoBallCount = m_NoBallMaxCount;
	m_KickBallMaxCount = 5;
	m_KickBallCount = 0;

	m_KickTopAngle = -5.0;
	m_KickRightAngle = -30.0;
	m_KickLeftAngle = 30.0;

	m_FollowMaxFBStep = 20.0;
    m_FollowMinFBStep = 5.0;
	m_FollowMaxRLTurn = 35.0;
	m_FitFBStep = 3.0;
	m_FitMaxRLTurn = 35.0;
	m_UnitFBStep = 0.3;
	m_UnitRLTurn = 1.0;

	m_GoalFBStep = 0;
	m_GoalRLTurn = 0;
	m_FBStep = 0;
	m_RLTurn = 0;
	DEBUG_PRINT = true;
	KickBall = 0;
}

BallFollower::~BallFollower()
{
}

void BallFollower::Process(Point2D ball_pos)
{
	if(DEBUG_PRINT == true)
		fprintf(stderr, "\r                                                                               \r");

    if(ball_pos.X == -1.0 || ball_pos.Y == -1.0)
    {
		KickBall = 0;

		if(m_NoBallCount > m_NoBallMaxCount)
		{
			int lagi=1;
			// can not find a ball
			m_GoalFBStep = 0;
			m_GoalRLTurn = 0;
			//Head::GetInstance()->MoveToHome();

			/*if((t>=-70)&&(t<=70)){t++;}
		
			if(t==70){t=-70; d+=10;}
			if(d==10){d=-25;lagi++;}*/
			if((d>=-30)&&(d<=10)){d++;}
		
			if(d==10||d>10){d=-30; t+=10;}
			if(t==70||t>70){t=-70;lagi++;}
			
			Head::GetInstance()->MoveByAngle(t,d);
			
			if(lagi==2){
				for(int i=0;i<5;i++)
				{
				Walking::GetInstance()->X_MOVE_AMPLITUDE = 15;
				Walking::GetInstance()->Start();
				lagi++;
				}
			}
			if(lagi==3){
				for(int i=0;i<5;i++)
				{
				Walking::GetInstance()->A_MOVE_AMPLITUDE = 8;
				Walking::GetInstance()->Start();
				lagi=1;
				}
			}

			if(DEBUG_PRINT == true)
				fprintf(stderr, "[NO BALL]");
		}
		else
		{
			m_NoBallCount++;
			if(DEBUG_PRINT == true)
				fprintf(stderr, "[NO BALL COUNTING(%d/%d)]", m_NoBallCount, m_NoBallMaxCount);
		}
    }
    else
    {
	if(arah == 1)//Gawang LAWAN di Selatan
	{
		m_NoBallCount = 0;		

		double pan = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
		double pan_range = Head::GetInstance()->GetLeftLimitAngle();
		double pan_percent = pan / pan_range;

		int kompas = MotionStatus::kompas;

		double tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
		double tilt_min = Head::GetInstance()->GetBottomLimitAngle();		
		double tilt_range = Head::GetInstance()->GetTopLimitAngle() - tilt_min;
		double tilt_percent = (tilt - tilt_min) / tilt_range;
		if(tilt_percent < 0)
			tilt_percent = -tilt_percent;

		if(pan > m_KickRightAngle && pan < m_KickLeftAngle)
		{
			if(tilt-5 <= (tilt_min + MX28::RATIO_VALUE2ANGLE))
			{
				if(ball_pos.Y < -100)
				{
					m_GoalFBStep = 0;
					m_GoalRLTurn = 0;

					if(m_KickBallCount >= m_KickBallMaxCount)
					{
						m_FBStep = 0;
						m_RLTurn = 0;						
						if(DEBUG_PRINT == true)
							fprintf(stderr, "[KICK]");
					    if((kompas>=800&&kompas<=900)||(kompas>=0&&kompas<=80))//utara
					  {
						if(pan > -11&&pan<15)
						{
							KickBall = 1; // Left
							if(DEBUG_PRINT == true)
								fprintf(stderr, " Left");
						}
						if(pan>=-20 && pan<-11)
						{
							KickBall = -1; // Right
							if(DEBUG_PRINT == true)
								fprintf(stderr, " Right");
						}
					  }
					  if(kompas>80&&kompas<=167)//timur
					  {
						if(pan>=-15 && pan<0)
						{
							KickBall = -2; // Right
							if(DEBUG_PRINT == true)
								fprintf(stderr, " sidekickRight");
						}
						else
						{
							KickBall = -4; // Left
							if(DEBUG_PRINT == true)
								fprintf(stderr, " geserkanan");
						}
					  }
					  if(kompas>167&&kompas<748)//selatan
					  {
						if(pan > 5&&pan<15)
						{
							KickBall = -3; // Left
							if(DEBUG_PRINT == true)
								fprintf(stderr, " putar kiri");
						}
						if(pan>=-15 && pan<0)
						{
							KickBall = 3; // Right
							if(DEBUG_PRINT == true)
								fprintf(stderr, " putar kanan");
						}
					  }
					  if(kompas>748&&kompas<800)//barat
					  {
						if(pan > 5&&pan<15)
						{
							KickBall = 2; // Left
							if(DEBUG_PRINT == true)
								fprintf(stderr, " sidekickLeft");
						}
						else
						{
							KickBall = 4; // Right
							if(DEBUG_PRINT == true)
								fprintf(stderr, " geserkanan");
						}
					  }
					}
					else
					{
						KickBall = 0;
						if(DEBUG_PRINT == true)
							fprintf(stderr, "[KICK COUNTING(%d/%d)]", m_KickBallCount, m_KickBallMaxCount);
					}
				}
				else
				{
					m_KickBallCount = 0;
					KickBall = 0;
					m_GoalFBStep = m_FitFBStep;
					m_GoalRLTurn = m_FitMaxRLTurn * pan_percent;
					if(DEBUG_PRINT == true)
						fprintf(stderr, "[FIT(P:%.2f T:%.2f>%.2f C:%3d)]", pan, ball_pos.Y, m_KickTopAngle,kompas);
				}
			}
			else
			{
				m_KickBallCount = 0;
				KickBall = 0;
				m_GoalFBStep = m_FollowMaxFBStep * tilt_percent;
				if(m_GoalFBStep < m_FollowMinFBStep)
				    m_GoalFBStep = m_FollowMinFBStep;
				m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent;
				if(DEBUG_PRINT == true)
					fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f C:%3d]", pan, tilt, tilt_min, kompas);
			}
		}
		else
		{
			m_KickBallCount = 0;
			KickBall = 0;
			m_GoalFBStep = 0;
			m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent;
			if(DEBUG_PRINT == true)
				fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f C:%3d]", pan, tilt, tilt_min, kompas);
		}
	   }//AKHIR GAWANG LAWAN DI SELATAN


	   if(arah == 2)//Gawang LAWAN di UTARA
	{
		m_NoBallCount = 0;		

		double pan = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
		double pan_range = Head::GetInstance()->GetLeftLimitAngle();
		double pan_percent = pan / pan_range;

		int kompas = MotionStatus::kompas;

		double tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
		double tilt_min = Head::GetInstance()->GetBottomLimitAngle();		
		double tilt_range = Head::GetInstance()->GetTopLimitAngle() - tilt_min;
		double tilt_percent = (tilt - tilt_min) / tilt_range;
		if(tilt_percent < 0)
			tilt_percent = -tilt_percent;

		if(pan > m_KickRightAngle && pan < m_KickLeftAngle)
		{
			if(tilt-2 <= (tilt_min + MX28::RATIO_VALUE2ANGLE))
			{
				if(ball_pos.Y < m_KickTopAngle+20)
				{
					m_GoalFBStep = 0;
					m_GoalRLTurn = 0;

					if(m_KickBallCount >= m_KickBallMaxCount)
					{
						m_FBStep = 0;
						m_RLTurn = 0;						
						if(DEBUG_PRINT == true)
							fprintf(stderr, "[KICK]");
					    if((kompas>748&&kompas<=900)||(kompas>=0&&kompas<=167))//utara
					  {
						if(pan > 5&&pan<15)
						{
							KickBall = -3; // Left
							if(DEBUG_PRINT == true)
								fprintf(stderr, " putar kiri");
						}
						if(pan>=-15 && pan<0)
						{
							KickBall = 3; // Right
							if(DEBUG_PRINT == true)
								fprintf(stderr, " putar kanan");
						}
					  }
					  if(kompas>167&&kompas<392)//timur
					  {
						if(pan>=-18 && pan<-10)
						{
							KickBall = 2; // Left
							if(DEBUG_PRINT == true)
								fprintf(stderr, " sidekickLeft");
						}
						else
						{
							KickBall = 4; // Right
							if(DEBUG_PRINT == true)
								fprintf(stderr, " geserkanan");
						}
					  }
					  if(kompas>=392&&kompas<=683)//selatan
					  {
						if(pan > 5&&pan<15)
						{
							KickBall = 1; // Left
							if(DEBUG_PRINT == true)
								fprintf(stderr, " Left");
						}
						if(pan>=-15 && pan<0)
						{
							KickBall = -1; // Right
							if(DEBUG_PRINT == true)
								fprintf(stderr, " Right");
						}						
					  }
					  if(kompas>683&&kompas<=748)//barat
					  {
						if(pan > 10&&pan<18)
						{
							KickBall = -2; // Right
							if(DEBUG_PRINT == true)
								fprintf(stderr, " sidekickRight");
						}
						else
						{
							KickBall = -4; // Left
							if(DEBUG_PRINT == true)
								fprintf(stderr, " geserkanan");
						}						
					  }
					}
					else
					{
						KickBall = 0;
						if(DEBUG_PRINT == true)
							fprintf(stderr, "[KICK COUNTING(%d/%d)]", m_KickBallCount, m_KickBallMaxCount);
					}
				}
				else
				{
					m_KickBallCount = 0;
					KickBall = 0;
					m_GoalFBStep = m_FitFBStep;
					m_GoalRLTurn = m_FitMaxRLTurn * pan_percent;
					if(DEBUG_PRINT == true)
						fprintf(stderr, "[FIT(P:%.2f T:%.2f>%.2f C:%3d)]", pan, ball_pos.Y, m_KickTopAngle,kompas);
				}
			}
			else
			{
				m_KickBallCount = 0;
				KickBall = 0;
				m_GoalFBStep = m_FollowMaxFBStep * tilt_percent;
				if(m_GoalFBStep < m_FollowMinFBStep)
				    m_GoalFBStep = m_FollowMinFBStep;
				m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent;
				if(DEBUG_PRINT == true)
					fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f C:%3d]", pan, tilt, tilt_min, kompas);
			}
		}
		else
		{
			m_KickBallCount = 0;
			KickBall = 0;
			m_GoalFBStep = 0;
			m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent;
			if(DEBUG_PRINT == true)
				fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f C:%3d]", pan, tilt, tilt_min, kompas);
		}
	   }//AKHIR GAWANG LAWAN DI UTARA		
	}

	if(m_GoalFBStep == 0 && m_GoalRLTurn == 0 && m_FBStep == 0 && m_RLTurn == 0)
	{
		if(Walking::GetInstance()->IsRunning() == true)
			Walking::GetInstance()->Stop();
		else
		{
			if(m_KickBallCount < m_KickBallMaxCount)
				m_KickBallCount++;
		}

		if(DEBUG_PRINT == true)
			fprintf(stderr, " STOP");
	}
	else
	{
		if(DEBUG_PRINT == true)
			fprintf(stderr, " START");

		if(Walking::GetInstance()->IsRunning() == false)
		{
			m_FBStep = 0;
			m_RLTurn = 0;
			m_KickBallCount = 0;
			KickBall = 0;
			Walking::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep;
			Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;
			Walking::GetInstance()->Start();			
		}
		else
		{
			if(m_FBStep < m_GoalFBStep)
				m_FBStep += m_UnitFBStep;
			else if(m_FBStep > m_GoalFBStep)
				m_FBStep = m_GoalFBStep;
			Walking::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep;

			if(m_RLTurn < m_GoalRLTurn)
				m_RLTurn += m_UnitRLTurn;
			else if(m_RLTurn > m_GoalRLTurn)
				m_RLTurn -= m_UnitRLTurn;
			Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;

			if(DEBUG_PRINT == true)
				fprintf(stderr, " (FB:%.1f RL:%.1f)", m_FBStep, m_RLTurn);
		}
	}	
}
