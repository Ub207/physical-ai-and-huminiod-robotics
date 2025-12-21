from sqlalchemy import create_engine, Column, Integer, String, DateTime, Text
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from datetime import datetime
from config.settings import settings
import logging
import os

logger = logging.getLogger(__name__)

Base = declarative_base()

class ChatSession(Base):
    __tablename__ = "chat_sessions"

    id = Column(Integer, primary_key=True, index=True)
    session_id = Column(String, unique=True, index=True)
    book_id = Column(String, index=True)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

class ChatMessage(Base):
    __tablename__ = "chat_messages"

    id = Column(Integer, primary_key=True, index=True)
    session_id = Column(String, index=True)
    role = Column(String)  # 'user' or 'assistant'
    content = Column(Text)
    timestamp = Column(DateTime, default=datetime.utcnow)

class DatabaseManager:
    def __init__(self):
        # Check if database URL is provided
        if not settings.neon_database_url or settings.neon_database_url == "postgresql://username:password@ep-xxx.us-east-1.aws.neon.tech/dbname?sslmode=require":
            logger.warning("Database configuration not provided. Using in-memory SQLite for demo.")
            # Use in-memory SQLite for demo purposes
            self.engine = create_engine("sqlite:///rag_chatbot_demo.db", echo=False)
        else:
            try:
                self.engine = create_engine(settings.neon_database_url, pool_pre_ping=True)
            except Exception as e:
                logger.error(f"Failed to connect to database: {e}. Using in-memory SQLite for demo.")
                self.engine = create_engine("sqlite:///rag_chatbot_demo.db", echo=False)

        self.SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=self.engine)
        try:
            Base.metadata.create_all(bind=self.engine)
        except Exception as e:
            logger.error(f"Failed to create database tables: {e}")

    def get_db(self):
        db = self.SessionLocal()
        try:
            yield db
        finally:
            db.close()

    def create_session(self, session_id: str, book_id: str):
        try:
            db = next(self.get_db())
            session = ChatSession(session_id=session_id, book_id=book_id)
            db.add(session)
            db.commit()
            db.refresh(session)
            return session
        except Exception as e:
            logger.error(f"Failed to create session: {e}")
            return None
        finally:
            try:
                db.close()
            except:
                pass

    def add_message(self, session_id: str, role: str, content: str):
        try:
            db = next(self.get_db())
            message = ChatMessage(session_id=session_id, role=role, content=content)
            db.add(message)
            db.commit()
            db.refresh(message)
            return message
        except Exception as e:
            logger.error(f"Failed to add message: {e}")
            return None
        finally:
            try:
                db.close()
            except:
                pass

    def get_session_history(self, session_id: str, limit: int = 10):
        try:
            db = next(self.get_db())
            messages = db.query(ChatMessage).filter(
                ChatMessage.session_id == session_id
            ).order_by(ChatMessage.timestamp.desc()).limit(limit).all()
            return list(reversed(messages))  # Return in chronological order
        except Exception as e:
            logger.error(f"Failed to get session history: {e}")
            return []
        finally:
            try:
                db.close()
            except:
                pass