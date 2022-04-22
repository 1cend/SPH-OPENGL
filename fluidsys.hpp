#pragma once
#define PI 3.1415926f
#include"grid.hpp"
#include"neighbortbl.hpp"
#include"point.hpp"
//流体系统类
class FluidSystem {
public:
	FluidSystem();
	void init(unsigned short maxPointCounts,
		const glm::vec3& wallBox_min, const glm::vec3& wallBox_max,
		const glm::vec3& initFluidBox_min, const glm::vec3& initFluidBox_max,
		const glm::vec3& gravity) {
		_init(maxPointCounts, fBox3(wallBox_min, wallBox_max),
			fBox3(initFluidBox_min, initFluidBox_max), gravity);
	}
	//获取点的尺寸（字节）
	unsigned int getPointStride()const { return sizeof(Point); }
	//获取点的数量
	unsigned int getPointCounts()const { return m_pointBuffer.size(); }
	//获取流体点缓存
	const Point* getPointBuf()const { return m_pointBuffer.get(0); }
	//逻辑帧
	void tick();
	void draw();
	~FluidSystem();
private:
	//初始化系统
	void _init(unsigned short maxPointCounts, const fBox3& wallBox,
		const fBox3& initFluidBox, const glm::vec3& gravity);
	//计算密度，压力以及相邻关系
	void _computerPressure();
	//计算加速度
	void _computerForce();
	//移动粒子
	void _advance();
	//创建初始液体块
	void _addFluidVolume(const fBox3& fluidBox, float spacing);

	//数据成员
	PointBuffer m_pointBuffer;
	GridContainer m_gridContainer;
	NeighborTable m_neighborTable;

	//点位置缓存数据(x,y,z)
	std::vector<float>posData;

	//SPH光滑核
	float m_kernelPoly6;
	float m_kernelSpiky;
	float m_kernelViscosity;

	//其他参数
	float m_pointDistance;//半径
	float m_unitScale;//尺寸单位
	float m_viscosity;//粘性
	float m_restDensity;//静态密度
	float m_pointMass;//质量
	float m_smoothRadius;//光滑核半径
	float m_gasConstantK;//气体常量k
	float m_boundaryStiffness;//边界刚性
	float m_boundaryDampening;//边界阻尼
	float m_speedLimiting;//速度限制
	glm::vec3 m_gravityDir;//重力矢量

	int m_rexSize[3];//网格尺寸

	fBox3 m_sphWallBox;
};

FluidSystem::FluidSystem() {
	m_unitScale = 0.004f;            // 尺寸单位
	m_viscosity = 1.0f;                // 粘度
	m_restDensity = 1000.f;            // 密度
	m_pointMass = 0.0006f;            // 粒子质量
	m_gasConstantK = 1.0f;                // 理想气体方程常量
	m_smoothRadius = 0.01f;            // 光滑核半径
	m_pointDistance = 0.0;

	m_rexSize[0] = 0;
	m_rexSize[1] = 0;
	m_rexSize[2] = 0;


	m_boundaryStiffness = 10000.f;
	m_boundaryDampening = 256;
	m_speedLimiting = 200;

	//Poly6 Kernel
	m_kernelPoly6 = 315.0f / (64.0f * PI * pow(m_smoothRadius, 9));
	//Spiky Kernel
	m_kernelSpiky = -45.0f / (PI * pow(m_smoothRadius, 6));
	//Viscosity Kernel
	m_kernelViscosity = 45.0f / (PI * pow(m_smoothRadius, 6));
}

FluidSystem::~FluidSystem()
{
}
//构造流体中的点
void FluidSystem::_addFluidVolume(const fBox3& fluidBox, float spacing) {
	for (float z = fluidBox.max.z; z >= fluidBox.min.z; z -= spacing)
	{
		for (float y = fluidBox.min.y; y <= fluidBox.max.y; y += spacing)
		{
			for (float x = fluidBox.min.x; x <= fluidBox.max.x; x += spacing)
			{
				Point* p = m_pointBuffer.addPointReuse();
				p->pos = glm::vec3(x, y, z);
			}
		}
	}
}

void FluidSystem::_init(unsigned short maxPointCounts, const fBox3& wallBox, const fBox3& initFluidBox, const glm::vec3& gravity) {
	m_pointBuffer.reset(maxPointCounts);
	m_sphWallBox = wallBox;
	m_gravityDir = gravity;
	m_pointDistance = pow(m_pointMass / m_restDensity, 1.0 / 3.0);//计算粒子间距
	_addFluidVolume(initFluidBox, m_pointDistance / m_unitScale);
	m_gridContainer.init(wallBox, m_unitScale, m_smoothRadius * 2.0, 1.0, m_rexSize);//设置网格尺寸(2r)
	posData = std::vector<float>(3 * m_pointBuffer.size(), 0);
}

void FluidSystem::_computerPressure() {
	float h2 = m_smoothRadius * m_smoothRadius;//h^2
	m_neighborTable.reset(m_pointBuffer.size());//重置邻接表
	for (unsigned int i = 0; i < m_pointBuffer.size(); i++) {
		Point* pi = m_pointBuffer.get(i);
		float sum = 0.0;
		m_neighborTable.point_prepare(i);
		int gridCell[8];
		m_gridContainer.findCells(pi->pos, m_smoothRadius / m_unitScale, gridCell);
		for (int cell = 0; cell < 8; cell++) {
			if (gridCell[cell] == -1) continue;
			int pndx = m_gridContainer.getGridData(gridCell[cell]);
			while (pndx != -1) {
				Point* pj = m_pointBuffer.get(pndx);
				if (pj == pi)sum += pow(h2, 3.0);
				else {
					glm::vec3 pi_pj = (pi->pos - pj->pos) * m_unitScale;
					float r2 = pi_pj.x * pi_pj.x + pi_pj.y * pi_pj.y + pi_pj.z * pi_pj.z;
					if (h2 > r2) {
						float h2_r2 = h2 - r2;
						sum += pow(h2_r2, 3.0);
						if (!m_neighborTable.point_add_neighbor(pndx, glm::sqrt(r2)))
							goto NEIGHBOR_FULL;
					}
				}
				pndx = pj->next;
			}
		}
	NEIGHBOR_FULL:
		m_neighborTable.point_commit();
		pi->density = m_kernelPoly6 * m_pointMass * sum;
		pi->pressure = (pi->density - m_restDensity) * m_gasConstantK;
	}
}

void FluidSystem::_computerForce() {
	float h2 = m_smoothRadius * m_smoothRadius;
	for (unsigned int i = 0; i < m_pointBuffer.size(); i++) {
		Point* pi = m_pointBuffer.get(i);
		glm::vec3 accel_sum = glm::vec3(0.0);
		int neighborCounts = m_neighborTable.getNeighborCounts(i);
		for (unsigned int j = 0; j < neighborCounts; j++) {
			unsigned short neighborIndex;
			float r;
			m_neighborTable.getNeighborInfo(i, j, neighborIndex, r);
			Point* pj = m_pointBuffer.get(neighborIndex);
			glm::vec3 ri_rj = (pi->pos - pj->pos) * m_unitScale;
			float h_r = m_smoothRadius - r;
			float pterm = -m_pointMass * m_kernelSpiky * h_r * h_r *
				(pi->pressure + pj->pressure) / (2.f * pi->density * pj->density);
			accel_sum += ri_rj * pterm / r;

			float vterm = m_kernelViscosity * m_viscosity * h_r *
				m_pointMass / (pi->density * pj->density);

			accel_sum += (pj->velocity_eval - pi->velocity_eval) * vterm;
		}
		pi->accel = accel_sum;
	}
}

void FluidSystem::_advance() {//粒子移动
	float deltaTime = 0.003;
	float SL2 = m_speedLimiting * m_speedLimiting;
	//posData.clear();
	for (unsigned int i = 0; i < m_pointBuffer.size(); i++) {
		Point* p = m_pointBuffer.get(i);
		glm::vec3 accel = p->accel;//获取p的加速度
		float accel2 = accel.x * accel.x + accel.y * accel.y + accel.z * accel.z;

		if (accel2 > SL2)//速度限制
			accel *= m_speedLimiting / glm::sqrt(accel2);

		float diff;
		//边界情况
		// Z方向边界
		diff = 1 * m_unitScale - (p->pos.z - m_sphWallBox.min.z) * m_unitScale;
		if (diff > 0.0f)
		{
			glm::vec3 norm(0, 0, 1.0);
			float adj = m_boundaryStiffness * diff - m_boundaryDampening * glm::dot(norm, p->velocity_eval);
			accel.x += adj * norm.x;
			accel.y += adj * norm.y;
			accel.z += adj * norm.z;
		}

		diff = 1 * m_unitScale - (m_sphWallBox.max.z - p->pos.z) * m_unitScale;
		if (diff > 0.0f)
		{
			glm::vec3 norm(0, 0, -1.0);
			float adj = m_boundaryStiffness * diff - m_boundaryDampening * glm::dot(norm, p->velocity_eval);
			accel.x += adj * norm.x;
			accel.y += adj * norm.y;
			accel.z += adj * norm.z;
		}

		//X方向边界
		diff = 1 * m_unitScale - (p->pos.x - m_sphWallBox.min.x) * m_unitScale;
		if (diff > 0.0f)
		{
			glm::vec3 norm(1.0, 0, 0);
			float adj = m_boundaryStiffness * diff - m_boundaryDampening * glm::dot(norm, p->velocity_eval);
			accel.x += adj * norm.x;
			accel.y += adj * norm.y;
			accel.z += adj * norm.z;
		}

		diff = 1 * m_unitScale - (m_sphWallBox.max.x - p->pos.x) * m_unitScale;
		if (diff > 0.0f)
		{
			glm::vec3 norm(-1.0, 0, 0);
			float adj = m_boundaryStiffness * diff - m_boundaryDampening * glm::dot(norm, p->velocity_eval);
			accel.x += adj * norm.x;
			accel.y += adj * norm.y;
			accel.z += adj * norm.z;
		}

		//Y方向边界
		diff = 1 * m_unitScale - (p->pos.y - m_sphWallBox.min.y) * m_unitScale;
		if (diff > 0.0f)
		{
			glm::vec3 norm(0, 1.0, 0);
			float adj = m_boundaryStiffness * diff - m_boundaryDampening * glm::dot(norm, p->velocity_eval);
			accel.x += adj * norm.x;
			accel.y += adj * norm.y;
			accel.z += adj * norm.z;
		}
		diff = 1 * m_unitScale - (m_sphWallBox.max.y - p->pos.y) * m_unitScale;
		if (diff > 0.0f)
		{
			glm::vec3 norm(0, -1.0, 0);
			float adj = m_boundaryStiffness * diff - m_boundaryDampening * glm::dot(norm, p->velocity_eval);
			accel.x += adj * norm.x;
			accel.y += adj * norm.y;
			accel.z += adj * norm.z;
		}

		// 重力作用
		accel += m_gravityDir;

		// 位置计算----------------------------
		glm::vec3 vnext = p->velocity + accel * deltaTime;            // v(t+1/2) = v(t-1/2) + a(t) dt
		p->velocity_eval = (p->velocity + vnext) * 0.5f;                // v(t+1) = [v(t-1/2) + v(t+1/2)] * 0.5        used to compute forces later
		p->velocity = vnext;
		p->pos += vnext * deltaTime / m_unitScale;        // p(t+1) = p(t) + v(t+1/2) dt

		//弹入位置数据
		posData[3 * i] = p->pos.x;
		posData[3 * i + 1] = p->pos.y;
		posData[3 * i + 2] = p->pos.z;
	}
}

void FluidSystem::tick() {
	m_gridContainer.insertParticles(&m_pointBuffer);//每帧刷新粒子位置
	_computerPressure();
	_computerForce();
	_advance();
	draw();
}

void FluidSystem::draw() {
	for (int i = 0; i < m_pointBuffer.size(); i++) {
		glPushMatrix();
		glTranslatef(posData[3 * i] * m_unitScale, posData[3 * i + 1] * m_unitScale, posData[3 * i + 2] * m_unitScale);
		glutSolidSphere(m_pointDistance, 30, 30);
		glPopMatrix();
	}

	glDisable(GL_LIGHTING);

	glColor3f(1.0f, 1.0f, 1.0f);

	// Draw bottom surface edges of the box
	glBegin(GL_LINE_LOOP);
	glVertex3f(m_sphWallBox.min.x * m_unitScale, m_sphWallBox.min.y * m_unitScale, m_sphWallBox.min.z * m_unitScale);
	glVertex3f(m_sphWallBox.max.x * m_unitScale, m_sphWallBox.min.y * m_unitScale, m_sphWallBox.min.z * m_unitScale);
	glVertex3f(m_sphWallBox.max.x * m_unitScale, m_sphWallBox.min.y * m_unitScale, m_sphWallBox.max.z * m_unitScale);
	glVertex3f(m_sphWallBox.min.x * m_unitScale, m_sphWallBox.min.y * m_unitScale, m_sphWallBox.max.z * m_unitScale);
	glEnd();

	// Draw top surface edges of the box
	glBegin(GL_LINE_LOOP);
	glVertex3f(m_sphWallBox.min.x * m_unitScale, m_sphWallBox.max.y * m_unitScale, m_sphWallBox.min.z * m_unitScale);
	glVertex3f(m_sphWallBox.max.x * m_unitScale, m_sphWallBox.max.y * m_unitScale, m_sphWallBox.min.z * m_unitScale);
	glVertex3f(m_sphWallBox.max.x * m_unitScale, m_sphWallBox.max.y * m_unitScale, m_sphWallBox.max.z * m_unitScale);
	glVertex3f(m_sphWallBox.min.x * m_unitScale, m_sphWallBox.max.y * m_unitScale, m_sphWallBox.max.z * m_unitScale);
	glEnd();

	// Draw left surface edges of the box
	glBegin(GL_LINE_LOOP);
	glVertex3f(m_sphWallBox.min.x * m_unitScale, m_sphWallBox.max.y * m_unitScale, m_sphWallBox.min.z * m_unitScale);
	glVertex3f(m_sphWallBox.min.x * m_unitScale, m_sphWallBox.max.y * m_unitScale, m_sphWallBox.max.z * m_unitScale);
	glVertex3f(m_sphWallBox.min.x * m_unitScale, m_sphWallBox.min.y * m_unitScale, m_sphWallBox.max.z * m_unitScale);
	glVertex3f(m_sphWallBox.min.x * m_unitScale, m_sphWallBox.min.y * m_unitScale, m_sphWallBox.min.z * m_unitScale);
	glEnd();

	// Draw right surface edges of the box
	glBegin(GL_LINE_LOOP);
	glVertex3f(m_sphWallBox.max.x * m_unitScale, m_sphWallBox.max.y * m_unitScale, m_sphWallBox.min.z * m_unitScale);
	glVertex3f(m_sphWallBox.max.x * m_unitScale, m_sphWallBox.max.y * m_unitScale, m_sphWallBox.max.z * m_unitScale);
	glVertex3f(m_sphWallBox.max.x * m_unitScale, m_sphWallBox.min.y * m_unitScale, m_sphWallBox.max.z * m_unitScale);
	glVertex3f(m_sphWallBox.max.x * m_unitScale, m_sphWallBox.min.y * m_unitScale, m_sphWallBox.min.z * m_unitScale);
	glEnd();

	glColor3f(1.0f, 0.0f, 0.0f);
	glEnable(GL_LIGHTING);
}