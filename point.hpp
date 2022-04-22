#pragma once
#include<bits/stdc++.h>
#include<GLFW/glfw3.h>
#include<GL/glut.h>
#include<glm/glm.hpp>
#include<glm/gtc/matrix_transform.hpp>
#include<glm/gtc/type_ptr.hpp>

#define ELEM_MAX 1e6
//���ӻ�����
struct Point {
	glm::vec3 pos;//��λ��
	float density;//�ܶ�
	float pressure;//ѹ��
	glm::vec3 accel;//���ٶ�
	glm::vec3 velocity;//�ٶ�
	glm::vec3 velocity_eval;//����ٶ�

	int next;//ָ����һ���������
};

class PointBuffer {
public:
	PointBuffer();
	void reset(unsigned int capacity);//�������ӵ㻺��
	unsigned int size()const { return m_fluidCounts; }//�������Ӹ���

	//��ȡ����Ϊindex�ĵ�
	Point* get(unsigned int index) { return m_FluidBuf + index; }
	const Point* get(unsigned int index) const { return m_FluidBuf + index; }

	Point* addPointReuse();//�����м����µĵ㲢����
	virtual ~PointBuffer();

private:
	Point* m_FluidBuf;//���ӵ㻺��
	unsigned int m_fluidCounts;//���Ӹ���
	unsigned int m_BufCapcity;//��������
};

PointBuffer::PointBuffer() :m_FluidBuf(0), m_fluidCounts(0), m_BufCapcity(0) {}
PointBuffer::~PointBuffer() {
	delete[] m_FluidBuf;
	m_FluidBuf = nullptr;
}

void PointBuffer::reset(unsigned int capacity) {
	m_BufCapcity = capacity;
	if (m_FluidBuf != 0) {//���㻺�治Ϊ�գ�����յ㻺��
		delete[] m_FluidBuf;
		m_FluidBuf = nullptr;
	}
	if (m_BufCapcity > 0)//���㻺�����ռ�
		m_FluidBuf = new Point[m_BufCapcity]();
	m_fluidCounts = 0;//���õ�����Ϊ0
}

Point* PointBuffer::addPointReuse() {
	if (m_fluidCounts >= m_BufCapcity) {
		if (m_BufCapcity * 2 > ELEM_MAX) {
			//����������ʱ���������ֵ
			return m_FluidBuf + m_fluidCounts - 1;
		}
		m_BufCapcity *= 2;
		Point* new_data = new Point[m_BufCapcity]();
		memcpy(new_data, m_FluidBuf, m_fluidCounts * sizeof(Point));
		delete[] m_FluidBuf;
		m_FluidBuf = new_data;
	}
	//�µĵ�
	Point* point = m_FluidBuf + m_fluidCounts++;
	point->pos = glm::vec3(0);
	point->next = 0;
	point->velocity = glm::vec3(0);
	point->velocity_eval = glm::vec3(0);
	point->pressure = 0;
	point->density = 0;
	point->accel = glm::vec3(0);

	return point;
}
